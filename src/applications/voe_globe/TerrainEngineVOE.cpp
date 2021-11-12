#include "TerrainEngineVOE.h"
#include "VoeImageUtils.h"

using namespace voe;

TerrainEngineVOE::WireframeInputHandler::WireframeInputHandler(vsg::ref_ptr<vsg::Switch>& switchNode)
    : switchNode(switchNode), state(0)
{
}

void TerrainEngineVOE::WireframeInputHandler::apply(vsg::KeyPressEvent& keyPress)
{
    vsg::ref_ptr<vsg::Switch> s(switchNode);
    if (!s)
        return;
    if (keyPress.keyBase == vsg::KEY_w)
    {
        state = (state + 1) % 3;
        s->setSingleChildOn(state);
    }
}


TerrainEngineVOE::TerrainEngineVOE()
    : tileReader(TileReaderVOE::create()), mipmapLevelsHint(16)
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

    vsg::dvec3 computeLatitudeLongitudeAltitude(double  lat, double lon, double alt)
    {
        return vsg::dvec3(lat, lon, alt);
    }
}

void TerrainEngineVOE::init(vsg::ref_ptr<vsg::Options> options, vsg::CommandLine& commandLine)
{
    tileReader->terrainEngine = this;
    commandLine.read("-t", tileReader->lodTransitionScreenHeightRatio);
    commandLine.read("-m", tileReader->maxLevel);
    // Read the osgEarth arguments
    tileReader->init(commandLine);

    osg::ArgumentParser parser = convertArgs(commandLine);
    if (!(mapNode  = osgEarth::MapNode::load(parser)))
    {
        throw std::runtime_error("no map");
    }
    osgEarth::Map* map = mapNode->getMap();
    auto const& em = map->getProfile()->getSRS()->getEllipsoid();
    ellipsoidModel = vsg::EllipsoidModel::create(em.getRadiusEquator(), em.getRadiusPolar());
    osgEarth::LayerVector layers;
    map->getLayers(layers);
    for (auto& layer : layers)
    {
        if (layer)
        {
            if (layer->getEnabled())
            {
                if (layer->getRenderType() == osgEarth::Layer::RENDERTYPE_TERRAIN_SURFACE)
                {
                    osgEarth::ImageLayer* ilayer = dynamic_cast<osgEarth::ImageLayer*>(layer.get());
                    if (ilayer)
                    {
                        imageLayer = ilayer;
                    }
                }
            }
        }
    }
    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr} // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };
    vsg::DescriptorSetLayoutBindings elevationDescriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT, nullptr},
        {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}, // normal texture
        {3, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }
    };
    if (!tileReader->getElevations())
    {
        descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);        
    }
    else
    {
        descriptorSetLayout = vsg::DescriptorSetLayout::create(elevationDescriptorBindings);
    }

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128} // projection view, and model matrices, actual push constant calls autoaatically provided by the VSG's DispatchTraversal
    };

    pipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout,
                                                                           simState.light_descriptorSetLayout},
        pushConstantRanges);

    sampler = vsg::Sampler::create();
    sampler->maxLod = mipmapLevelsHint;
    sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->anisotropyEnable = VK_TRUE;
    sampler->maxAnisotropy = 16.0f;

    elevationSampler = vsg::Sampler::create();
    elevationSampler->maxLod = mipmapLevelsHint;
    elevationSampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    elevationSampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    elevationSampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

    normalSampler = vsg::Sampler::create();
    normalSampler->maxLod = mipmapLevelsHint;
    normalSampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    normalSampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    normalSampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    
}

vsg::ref_ptr<vsg::Node> TerrainEngineVOE::createScene(vsg::ref_ptr<vsg::Options> options)
{
        // set up search paths to SPIRV shaders and textures
    vsg::Paths searchPaths = vsg::getEnvPaths("VSG_FILE_PATH");

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
    if (tileReader->getReverseDepth())
    {
        vsg::ShaderStage::SpecializationConstants specializationConstants{
        {0, vsg::uintValue::create(1)}
        };
        vertexShader->specializationConstants = specializationConstants;
        depthStencilState->depthCompareOp = VK_COMPARE_OP_GREATER;
    }
    vsg::GraphicsPipelineStates fillPipelineStates{
        vsg::RasterizationState::create(),
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::MultisampleState::create(),
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
    auto switchRoot = vsg::Switch::create();
    auto lightStateGroup = vsg::StateGroup::create();
    auto lightDescriptorSet = vsg::DescriptorSet::create(simState.light_descriptorSetLayout,
                                                         vsg::Descriptors{simState.lightValues});
    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, lightDescriptorSet);
    bindDescriptorSet->slot = 2; // XXX Why?
    lightStateGroup->add(bindDescriptorSet);
    vsg::GraphicsPipelineStates* pipelineStates[] = {&fillPipelineStates, &wirePipelineStates, &pointPipelineStates};
    for (int i = 0; i < 3; ++i)
    {
        auto graphicsPipeline = vsg::GraphicsPipeline::create(pipelineLayout, shaderStages, *pipelineStates[i]);
        auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);
        auto stateGroup = vsg::StateGroup::create();
        stateGroup->add(bindGraphicsPipeline);
        stateGroup->addChild(lightStateGroup);
        switchRoot->addChild(i == 0, stateGroup);
    }
    auto plodRoot = vsg::read_cast<vsg::Node>("root.tile", options);
    lightStateGroup->addChild(plodRoot);
    // assign the EllipsoidModel so that the overall geometry of the database can be used as guide for clipping and navigation.
    switchRoot->setObject("EllipsoidModel", ellipsoidModel);
    sceneRootSwitch = switchRoot;

    return switchRoot;
 }

vsg::ref_ptr<vsg::Node>
TerrainEngineVOE::createTile(const osgEarth::TileKey& key, vsg::ref_ptr<const vsg::Options> options) const
{
    vsg::ref_ptr<vsg::Node> nullNode;
    osgEarth::GeoImage gimage = imageLayer->createImage(key);
    osg::ref_ptr<osgEarth::ElevationTexture> elevationTexture;
    // create StateGroup to bind any texture state
    if (!gimage.valid())
    {
        return nullNode;
    }
    auto scenegraph = vsg::StateGroup::create();
    auto data = convertToVsg(gimage.getImage(), true);
    const osgEarth::GeoExtent& extent = gimage.getExtent();
    osgEarth::GeoPoint centroid = extent.getCentroid();
    auto localToWorld = ellipsoidModel->computeLocalToWorldTransform(vsg::dvec3(centroid.y(), centroid.x(), 0.0));
    auto worldToLocal = vsg::inverse(localToWorld);

        // create texture image and associated DescriptorSets and binding
    auto texDescriptor = vsg::DescriptorImage::create(sampler, data, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    vsg::ref_ptr<vsg::DescriptorSet> descriptorSet;
    if (elevations)
    {
        bool elevResult = mapNode->getMap()->getElevationPool()->getTile(key, false, elevationTexture,
                                                                             nullptr, nullptr);
        if (!elevResult)
        {
            return nullNode;
        }
        elevationTexture->generateNormalMap(mapNode->getMap(), nullptr, nullptr);
        auto elevData = convertToVsg(elevationTexture->getImage());
        auto normalData = convertToVsg(elevationTexture->getNormalMapTexture()->getImage());
        auto elevationTexDescriptor = vsg::DescriptorImage::create(elevationSampler, elevData, 1, 0,
                                                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        auto normalTexDescriptor = vsg::DescriptorImage::create(normalSampler, normalData, 2, 0,
                                                          VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        auto tileParams = TileParamsValue::create();
        // From engine_rex/TerrainCuller.cpp
        float bias = .5;
        float tileWidth = static_cast<float>(elevData->width());
        tileParams->value().elevTexelCoeff.set((tileWidth - (2.0*bias)) / tileWidth, bias / tileWidth);
        // matrices are identity.
        tileParams->value().elevationTexMatrix = vsg::mat4();
        tileParams->value().normalTexMatrix = vsg::mat4();
        // XXX Sucks that you need to specify the "binding" (location) in the descriptor buffer itself.
        auto tileParamsBuffer = vsg::DescriptorBuffer::create(tileParams, 3);
        descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout,
                                                   vsg::Descriptors{texDescriptor, elevationTexDescriptor,
                                                       normalTexDescriptor, tileParamsBuffer});
    }
    else
    {
        descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{texDescriptor});
    }
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, vsg::DescriptorSets{descriptorSet});
    scenegraph->add(bindDescriptorSets);

    // set up model transformation node
    auto transform = vsg::MatrixTransform::create(localToWorld); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    uint32_t numRows = 32;
    uint32_t numCols = 32;
    uint32_t numVertices = numRows * numCols;
    uint32_t numTriangles = (numRows - 1) * (numCols - 1) * 2;

    double longitudeOrigin = extent.xMin();
    double longitudeScale = (extent.xMax() - extent.xMin()) / double(numCols - 1);
    double latitudeOrigin = extent.yMin();
    double latitudeScale = (extent.yMax() - extent.yMin()) / double(numRows - 1);

    float sCoordScale = 1.0f / float(numCols - 1);
    float tCoordScale = 1.0f / float(numRows - 1);
    float tCoordOrigin = 0.0;

    if (data->getLayout().origin == vsg::TOP_LEFT)
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
        for (uint32_t c = 0; c < numCols; ++c)
        {
            vsg::dvec3 lla
                = computeLatitudeLongitudeAltitude(latitudeOrigin + double(r) * latitudeScale,
                                                   longitudeOrigin + double(c) * longitudeScale,
                                                   0.0);
            const float latitude = vsg::radians(lla[0]);
            const float longitude = vsg::radians(lla[1]);

            auto ecef = ellipsoidModel->convertLatLongAltitudeToECEF(lla);
            vsg::vec3 vertex(worldToLocal * ecef);
            vsg::vec2 texcoord(float(c) * sCoordScale, tCoordOrigin + float(r) * tCoordScale);
            vsg::dvec4 worldUp(cos(longitude) * cos(latitude),
                               sin(longitude) * cos(latitude),
                               sin(latitude),
                               0.0);
            vsg::dvec4 localUp(worldToLocal * worldUp);
            uint32_t vi = c + r * numCols;
            vertices->set(vi, vertex);
            colors->set(vi, color);
            texcoords->set(vi, texcoord);
            ellipsoidNormals->set(vi, vsg::vec3(localUp[0], localUp[1], localUp[2]));
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

    // add drawCommands to transform
    transform->addChild(drawCommands);
    return scenegraph;
}


vsg::ref_ptr<TerrainEngineVOE::WireframeInputHandler> TerrainEngineVOE::createWireframeHandler()
{
    return WireframeInputHandler::create(sceneRootSwitch);
}
