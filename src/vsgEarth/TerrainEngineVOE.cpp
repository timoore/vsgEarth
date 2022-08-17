#include "TerrainEngineVOE.h"
#include "VoeImageUtils.h"
#include "vsgEarth/Config.h"

#include "osgEarth/Elevation"
#include "osgEarth/ImageLayer"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <osgEarth/Locators>
#include <vector>
#include <vsg/all.h>

using namespace osgEarth;

TerrainEngineVOE::WireframeInputHandler::WireframeInputHandler(vsg::ref_ptr<vsg::StateSwitch>& switchNode)
    : switchNode(switchNode), state(0)
{
}

void TerrainEngineVOE::WireframeInputHandler::apply(vsg::KeyPressEvent& keyPress)
{
    vsg::ref_ptr<vsg::StateSwitch> s(switchNode);
    if (!s)
        return;
    if (keyPress.keyBase == vsg::KEY_w)
    {
        state = (state + 1) % 3;
        for (unsigned i = 0; i < 3; ++i)
        {
            s->children[i].mask = i == state;
        }
    }
}

void VOELayerCallback::onVisibleChanged(class VisibleLayer *layer)
{
    layerParams->setEnabled(layerIdx, layer->getVisible());
}

void VOELayerCallback::onOpacityChanged(class VisibleLayer *layer)
{
    layerParams->setOpacity(layerIdx, layer->getOpacity());
}

TerrainEngineVOE::TerrainEngineVOE()
    : tileReader(TileReaderVOE::create()), mipmapLevelsHint(16), numImageLayers(0),
      samples(VK_SAMPLE_COUNT_1_BIT)
{
    // XXX I'm not clear on what would happen if one tries to create an observer_ptr in an
    // unreferenced object, like TerrainEngineVOE is inside this constructor. Therefore, defer
    // initializing tileReader's observer_ptr to this.
}

namespace
{
    osg::ArgumentParser convertArgs(vsg::CommandLine& commandLine)
    {
        return osg::ArgumentParser(&commandLine.argc(), commandLine.argv());
    }
}

osgEarth::MapNode*
TerrainEngineVOE::init(vsg::ref_ptr<vsg::Options>, vsg::CommandLine& commandLine,
                       vsg::ref_ptr<vsg::WindowTraits> traits)
{
    tileReader->terrainEngine = this;
    // Read the osgEarth arguments
    tileReader->init(commandLine);
    if (traits.valid())
    {
        traits->depthFormat = VK_FORMAT_D32_SFLOAT;
        traits->deviceFeatures->get().fillModeNonSolid = VK_TRUE;
    }
    osg::ArgumentParser parser = convertArgs(commandLine);
    if (!(mapNode  = osgEarth::MapNode::load(parser)))
    {
        return nullptr;
    }
    else
    {
        return mapNode.get();
    }
}

vsg::ref_ptr<vsg::Node> TerrainEngineVOE::createScene(vsg::ref_ptr<vsg::Options> options)
{
    if (!mapNode)
        throw std::runtime_error("no map");
    const osgEarth::MapNode::Options& mapNodeOptions = const_cast<const osgEarth::MapNode*>(mapNode.get())->options();
    modelFactory = new osgEarth::TerrainTileModelFactory(mapNodeOptions.terrain().get());
    osgEarth::Map* map = mapNode->getMap();
    auto const& em = map->getProfile()->getSRS()->getEllipsoid();
    ellipsoidModel = vsg::EllipsoidModel::create(em.getRadiusEquator(), em.getRadiusPolar());

    // set up graphics pipeline

    vsg::DescriptorSetLayoutBindings layerDescriptorBindings{
        // layer parameters
        { 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }
        };

    auto layerDescriptorSetLayout = vsg::DescriptorSetLayout::create(layerDescriptorBindings);

    // Now the visible layers
        LayerVector layers;
    map->getLayers(layers);
    std::vector<ImageLayer*> imageLayers;
    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        Layer* layer = i->get();

        if (!layer->isOpen())
            continue;

        if (layer->getRenderType() != layer->RENDERTYPE_TERRAIN_SURFACE)
            continue;

        ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);
        if (imageLayer)
        {
            imageLayers.push_back(imageLayer);
        }
    }
    numImageLayers= imageLayers.size();
    layerParams = LayerParams::create(numImageLayers);
    for (int i = 0; i < numImageLayers; ++i)
    {
        layerParams->layerUIDs[i] = imageLayers[i]->getUID();
        layerParams->setEnabled(i, imageLayers[i]->getVisible());
        layerParams->setOpacity(i, imageLayers[i]->getOpacity());
        layerParams->setBlendMode(i, imageLayers[i]->getColorBlending());
        imageLayers[i]->Layer::addCallback(new VOELayerCallback(layerParams, i));
    }

    vsg::DescriptorSetLayoutBindings elevationDescriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, numImageLayers, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        // Elevation texture
        {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT, nullptr},
        // normal texture
        {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        {3, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }
    };
    descriptorSetLayout = vsg::DescriptorSetLayout::create(elevationDescriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128} // projection view, and model matrices, actual push constant calls autoaatically provided by the VSG's DispatchTraversal
    };

    pipelineLayout
        = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout,
                                                                vsg::ViewDescriptorSetLayout::create(),
                                                                layerDescriptorSetLayout},
            pushConstantRanges);

    sampler = vsg::Sampler::create();
    sampler->maxLod = mipmapLevelsHint;
    sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->anisotropyEnable = VK_TRUE;
    sampler->maxAnisotropy = 16.0f;

    elevationSampler = vsg::Sampler::create();
    elevationSampler->maxLod = 0;
    elevationSampler->minFilter = VK_FILTER_NEAREST;
    elevationSampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    elevationSampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    elevationSampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

    normalSampler = vsg::Sampler::create();
    normalSampler->maxLod = 0;
    normalSampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    normalSampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    normalSampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

    osg::Texture* emptyOsgElevation = createEmptyElevationTexture();
    auto emptyElevData = convertToVsg(emptyOsgElevation->getImage(0));
    emptyElevationDescImage = vsg::DescriptorImage::create(elevationSampler, emptyElevData, 1, 0,
                                                           VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    osg::Texture* emptyOsgNormal = createEmptyNormalMapTexture();
    auto emptyNormalData = convertToVsg(emptyOsgNormal->getImage(0));
    emptyNormalDescImage = vsg::DescriptorImage::create(normalSampler, emptyNormalData, 2, 0,
                                                        VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    // set up search paths to SPIRV shaders and textures
    vsg::Paths searchPaths = vsg::getEnvPaths("VSG_FILE_PATH");
    auto dataDirPath = vsg::Path(VSGEARTH_FULL_DATA_DIR);
    searchPaths.push_back(dataDirPath);
    // load shaders
        vsg::ref_ptr<vsg::ShaderStage> vertexShader = vsg::ShaderStage::read(VK_SHADER_STAGE_VERTEX_BIT, "main", vsg::findFile("elevation.vert.spv", searchPaths));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = vsg::ShaderStage::read(VK_SHADER_STAGE_FRAGMENT_BIT, "main", vsg::findFile("elevation.frag.spv", searchPaths));
    if (!vertexShader || !fragmentShader)
    {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, // colour data
        VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX},  // tex coord data
        VkVertexInputBindingDescription{3, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}  // normals
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}, // colour data
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0},    // tex coord data
        VkVertexInputAttributeDescription{3, 3, VK_FORMAT_R32G32B32_SFLOAT, 0} // vertex data
    };

    auto depthStencilState = vsg::DepthStencilState::create();
    vsg::ShaderStage::SpecializationConstants specializationConstants{
        {0, vsg::uintValue::create(reverseDepth)},
        {1, vsg::uintValue::create(numImageLayers)}
    };
    vertexShader->specializationConstants = specializationConstants;
    fragmentShader->specializationConstants = specializationConstants;
    vsg::GraphicsPipelineStates fillPipelineStates{
        vsg::RasterizationState::create(),
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::MultisampleState::create(samples),
        vsg::ColorBlendState::create(),
        depthStencilState};

    auto wireRasterState = vsg::RasterizationState::create();
    wireRasterState->polygonMode = VK_POLYGON_MODE_LINE;
    vsg::GraphicsPipelineStates wirePipelineStates(fillPipelineStates);
    wirePipelineStates[0] = wireRasterState;
    auto pointRasterState = vsg::RasterizationState::create();
    pointRasterState->polygonMode = VK_POLYGON_MODE_POINT;
    vsg::GraphicsPipelineStates pointPipelineStates(fillPipelineStates);
    pointPipelineStates[0] = pointRasterState;

    vsg::ShaderStages shaderStages{vertexShader, fragmentShader};
    auto lightStateGroup = vsg::StateGroup::create();
    lightStateGroup->add(vsg::BindViewDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                                                             1));
    auto layerDescriptorSet
        = vsg::DescriptorSet::create(layerDescriptorSetLayout,
                                     vsg::Descriptors{layerParams->layerParamsDescriptor});
    auto bindLayerDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                                                                 2, layerDescriptorSet);
    lightStateGroup->add(bindLayerDescriptorSet);
    // StateSwitch experiments
    auto rasterStateGroup = vsg::StateGroup::create();
     rasterSwitch = vsg::StateSwitch::create();
    vsg::GraphicsPipelineStates* pipelineStates[]
        = {&fillPipelineStates, &wirePipelineStates, &pointPipelineStates};
    for (int i = 0; i < 3; ++i)
    {
        auto graphicsPipeline
            = vsg::GraphicsPipeline::create(pipelineLayout, shaderStages, *pipelineStates[i]);
        auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);
        rasterSwitch->add(i == 0, bindGraphicsPipeline);
    }
    rasterStateGroup->add(rasterSwitch);
    rasterStateGroup->addChild(lightStateGroup);
    auto plodRoot = vsg::read_cast<vsg::Node>("root.tile", options);
    lightStateGroup->addChild(plodRoot);
    // Put everything under a group node in order to have a place to hang the lights.
    auto sceneGroup = vsg::Group::create();
    sceneGroup->addChild(rasterStateGroup);
    directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "directional";
    directionalLight->color = simState.getColor();
    directionalLight->intensity = 1.0f;
    directionalLight->direction = simState.worldDirection;
    sceneGroup->addChild(directionalLight);
    ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color = simState.getAmbient();
    ambientLight->intensity = 1.0f;
    sceneGroup->addChild(ambientLight);
    // assign the EllipsoidModel so that the overall geometry of the database can be used as guide
    // for clipping and navigation.
    sceneGroup->setObject("EllipsoidModel", ellipsoidModel);
    return sceneGroup;
 }

vsg::ref_ptr<vsg::Commands> createTileGeometry(const osgEarth::TileKey& tileKey, uint32_t tileSize,
                                               uint8_t textureOrigin)
{
    const uint32_t numRows = tileSize;
    const uint32_t numCols = tileSize;
    const uint32_t numVertices = numRows * numCols;
    const uint32_t numTriangles = (numRows - 1) * (numCols - 1) * 2;
    osgEarth::GeoLocator locator(tileKey.getExtent());
    osgEarth::GeoPoint centroid = tileKey.getExtent().getCentroid();
    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal(world2local);
    local2world.invert(world2local);

    float sCoordScale = 1.0f / float(numCols - 1);
    float tCoordScale = 1.0f / float(numRows - 1);
    float tCoordOrigin = 0.0;

    if (textureOrigin == vsg::TOP_LEFT)
    {
        tCoordScale = -tCoordScale;
        tCoordOrigin = 1.0f;
    }

    vsg::vec3 color(1.0f, 1.0f, 1.0f);

    // set up vertex coords
    auto vertices = vsg::vec3Array::create(numVertices);
    auto colors = vsg::vec3Array::create(numVertices);
    auto texcoords = vsg::vec2Array::create(numVertices);
    auto ellipsoidNormals = vsg::vec3Array::create(numVertices);
    for (uint32_t r = 0; r < numRows; ++r)
    {
        double ny = r / static_cast<double>(numRows - 1);
        for (uint32_t c = 0; c < numCols; ++c)
        {
            double nx = c / static_cast<double>(numCols - 1);
            uint32_t vi = c + r * numCols;
            osg::Vec3d unit(nx, ny, 0.0);
            osg::Vec3d model;
            locator.unitToWorld(unit, model);
            // OSG multiplication order
            osg::Vec3d modelLTP(model * world2local);
            vertices->set(vi, vsg::vec3(toVsg(modelLTP)));
            unit.z() = 1.0f;
            osg::Vec3d modelPlusOne;
            locator.unitToWorld(unit, modelPlusOne);
            osg::Vec3d normal;
            normal = (modelPlusOne*world2local) - modelLTP;
            normal.normalize();
            ellipsoidNormals->set(vi, vsg::vec3(toVsg(normal)));
            vsg::vec2 texcoord(c * sCoordScale, tCoordOrigin + r * tCoordScale);
            texcoords->set(vi, texcoord);
            colors->set(vi, color);
        }
    }

    // set up indices
    auto indices = vsg::ushortArray::create(numTriangles * 3);
    auto itr = indices->begin();
    for (uint32_t r = 0; r < numRows - 1; ++r)
    {
        for (uint32_t c = 0; c < numCols - 1; ++c)
        {
            uint32_t vi = c + r * numCols;
            (*itr++) = vi;
            (*itr++) = vi + 1;
            (*itr++) = vi + numCols;
            (*itr++) = vi + numCols;
            (*itr++) = vi + 1;
            (*itr++) = vi + numCols + 1;
        }
    }

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors, texcoords,
                                                                           ellipsoidNormals}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(indices->size(), 1, 0, 0, 0));
    return drawCommands;
}

// per-tile image layer data
struct ImageLayerTileData
{
    ImageLayerTileData(vsg::ref_ptr<vsg::ImageInfo> info,
                       osg::Matrixf& texMatrix,
                       UID uid)
        : info(info), texMatrix(texMatrix), uid(uid)
    {}
    vsg::ref_ptr<vsg::ImageInfo> info;
    osg::Matrixf& texMatrix;
    UID uid;
};

vsg::ref_ptr<vsg::Node>
TerrainEngineVOE::createTile(const osgEarth::TileKey& key, vsg::ref_ptr<const vsg::Options> options) const
{
    osg::ref_ptr<osgEarth::TerrainTileModel> tileModel
        = modelFactory->createTileModel(getMap(), key, osgEarth::CreateTileManifest(),
                                        nullptr, nullptr);
    std::vector<ImageLayerTileData> tileData;
    uint8_t imageOrigin = vsg::BOTTOM_LEFT; // XXX huge hack; should probably just decide on
    // OpenGL order
    for (auto& colorLayer : tileModel->colorLayers())
    {
        auto& layer = colorLayer.layer();
        if (!layer.valid())
            continue;
        ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer.get());
        if (imageLayer && colorLayer.texture()->osgTexture().valid())
        {
            auto texture = colorLayer.texture()->osgTexture().get();
            osg::Matrixf& textureMatrix = colorLayer.matrix();
            auto data = convertToVsg(texture->getImage(0), true);
            imageOrigin = data->getLayout().origin;
            tileData.push_back(ImageLayerTileData(vsg::ImageInfo::create(sampler, data), textureMatrix,
                                                  imageLayer->getUID()));
        }
    }
    if (tileData.empty())
        return {};
    // create StateGroup to bind any texture state
    auto scenegraph = vsg::StateGroup::create();
    osgEarth::GeoPoint centroid = key.getExtent().getCentroid();
    auto localToWorld = ellipsoidModel->computeLocalToWorldTransform(vsg::dvec3(centroid.y(), centroid.x(), 0.0));

    // create texture image and associated DescriptorSets and binding
    vsg::ImageInfoList imageTextures(tileData.size());
    std::transform(tileData.begin(), tileData.end(), imageTextures.begin(),
                   [](const ImageLayerTileData& data) { return data.info; });
    auto texDescriptor = vsg::DescriptorImage::create(imageTextures, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    vsg::ref_ptr<vsg::DescriptorSet> descriptorSet;
    osg::ref_ptr<osgEarth::ElevationTexture> elevationTexture;
    vsg::ref_ptr<vsg::DescriptorImage> elevationTexDescriptor = emptyElevationDescImage;
    vsg::ref_ptr<vsg::DescriptorImage> normalTexDescriptor = emptyNormalDescImage;
    const float bias = .5;
    float tileWidth = 1.0f;
    if (tileModel->elevation().texture())
    {
        auto osgElevationTexture = tileModel->elevation().texture()->osgTexture();
        auto elevData = convertToVsg(osgElevationTexture->getImage(0));
        auto osgNormalTexture = tileModel->normalMap().texture()->osgTexture();
        auto normalData = convertToVsg(osgNormalTexture->getImage(0));
        elevationTexDescriptor = vsg::DescriptorImage::create(elevationSampler, elevData, 1, 0,
                                                              VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        normalTexDescriptor = vsg::DescriptorImage::create(normalSampler, normalData, 2, 0,
                                                           VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        tileWidth = static_cast<float>(elevData->width());
    }
    TileParams tileParams(numImageLayers);
    for (int i = 0; i < numImageLayers; ++i)
    {
        UID uid = tileData[i].uid;
        tileParams.imageTexMatrix(i) = toVsg(tileData[i].texMatrix);
        auto layerItr = std::find(layerParams->layerUIDs.begin(), layerParams->layerUIDs.end(), uid);
        if (layerItr != layerParams->layerUIDs.end())
        {
            tileParams.layerIndex(i) = std::distance(layerParams->layerUIDs.begin(),layerItr);
        }
        else
        {
            tileParams.layerIndex(i) = 0; // XXX Index into layerParams
        }
    }
    if (tileModel->elevation().texture())
    {
        tileParams.elevationTexMatrix()
            = toVsg((tileModel->elevation().matrix()));
    }
    {
        tileParams.elevationTexMatrix() = vsg::mat4();
    }
    tileParams.elevTexelCoeff()[0] = (tileWidth - (2.0*bias)) / tileWidth;
    tileParams.elevTexelCoeff()[1] = bias / tileWidth;
    tileParams.numImageLayers() = numImageLayers;

    // XXX Sucks that you need to specify the "binding" (location) in the descriptor buffer itself.
    auto tileParamsBuffer = vsg::DescriptorBuffer::create(tileParams.data, 3);
    descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout,
                                               vsg::Descriptors{texDescriptor, elevationTexDescriptor,
                                                   normalTexDescriptor, tileParamsBuffer});
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, vsg::DescriptorSets{descriptorSet});
    scenegraph->add(bindDescriptorSets);

    // set up model transformation node
    auto transform = vsg::MatrixTransform::create(localToWorld); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);
    auto drawCommands = createTileGeometry(key, 32, imageOrigin);
    // add drawCommands to transform
    transform->addChild(drawCommands);
    return scenegraph;
}


vsg::ref_ptr<TerrainEngineVOE::WireframeInputHandler> TerrainEngineVOE::createWireframeHandler()
{
    return WireframeInputHandler::create(rasterSwitch);
}

void TerrainEngineVOE::update(vsg::ref_ptr<vsg::Viewer> viewer, vsg::ref_ptr<vsg::Camera> camera)
{
    // XXX Check if this is still needed
    viewer->waitForFences(1, 50000000);
    directionalLight->color = simState.getColor();
    directionalLight->direction = simState.worldDirection;
    ambientLight->color = simState.getAmbient();
    layerParams->layerParamsDescriptor->copyDataListToBuffers();
}
