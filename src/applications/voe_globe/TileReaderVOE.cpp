#include "TileReaderVOE.h"
#include "VoeImageUtils.h"
#include "VoeLOD.h"

#include <osgEarth/Notify>
#include <stdexcept>

using namespace voe;

osg::ArgumentParser convertArgs(vsg::CommandLine& commandLine)
{
    return osg::ArgumentParser(&commandLine.argc(), commandLine.argv());
}

// We're not rendering Mercator projections here, so skip this

namespace
{
vsg::dvec3 computeLatitudeLongitudeAltitude(double  lat, double lon, double alt)
{
    return vsg::dvec3(lat, lon, alt);
}
}

vsg::ref_ptr<vsg::Object> TileReaderVOE::read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options) const
{
    auto extension = vsg::fileExtension(filename);
    if (extension != "tile") return {};

    std::string tile_info = filename.substr(0, filename.length() - 5);
    if (tile_info == "root")
    {
        return read_root(options);
    }
    else
    {
        std::stringstream sstr(tile_info);

        uint32_t x, y, lod;
        char ch;                // the '/'
        sstr >> lod >> ch >> x >> ch >> y;
        osgEarth::TileKey key(lod, x, y, mapNode->getMap()->getProfile());

        // std::cout<<"read("<<filename<<") -> tile_info = "<<tile_info<<", x = "<<x<<", y = "<<y<<", z = "<<lod<<std::endl;

        return read_subtile(key, options);
    }
}

vsg::ref_ptr<vsg::Object> TileReaderVOE::read_root(vsg::ref_ptr<const vsg::Options> options) const
{
    // State group for the whole terrain
    auto group = createRoot();
    std::vector<osgEarth::TileKey> rootKeys;
    mapNode->getMap()->getProfile()->getRootKeys(rootKeys);
    for (const auto& key : rootKeys)
    {
        auto tile = createTile(key, options);
        if (tile)
        {
            vsg::ComputeBounds computeBound;
            tile->accept(computeBound);
            auto& bb = computeBound.bounds;
            vsg::dsphere newBound((bb.min.x + bb.max.x) * 0.5, (bb.min.y + bb.max.y) * 0.5, (bb.min.z + bb.max.z) * 0.5, vsg::length(bb.max - bb.min) * 0.5);

            auto plod = VoeLOD::create();
            plod->bound = newBound;
            plod->children[0] = VoeLOD::Child{0.25, {}};  // external child visible when it's bound occupies more than 1/4 of the height of the window
            plod->children[1] = VoeLOD::Child{0.0, tile}; // visible always
            plod->filename = vsg::make_string(key.str(), ".tile");
            plod->options = options;

            group->addChild(plod);
        }
    }

    uint32_t maxLevel = 20;
    uint32_t estimatedNumOfTilesBelow = 0;
    uint32_t maxNumTilesBelow = 40000;

    uint32_t level = 0;
    for (uint32_t i = level; i < maxLevel; ++i)
    {
        estimatedNumOfTilesBelow += std::pow(4, i - level);
    }

    uint32_t tileMultiplier = std::min(estimatedNumOfTilesBelow, maxNumTilesBelow) + 1;

    // set up the ResourceHints required to make sure the VSG preallocates enough Vulkan resources for the paged database
    vsg::CollectDescriptorStats collectStats;
    group->accept(collectStats);

    auto resourceHints = vsg::ResourceHints::create();

    resourceHints->maxSlot = collectStats.maxSlot;
    resourceHints->numDescriptorSets = static_cast<uint32_t>(collectStats.computeNumDescriptorSets() * tileMultiplier);
    resourceHints->descriptorPoolSizes = collectStats.computeDescriptorPoolSizes();

    for (auto& poolSize : resourceHints->descriptorPoolSizes)
    {
        poolSize.descriptorCount = poolSize.descriptorCount * tileMultiplier;
    }

    group->setObject("ResourceHints", resourceHints);

    // assign the EllipsoidModel so that the overall geometry of the database can be used as guide for clipping and navigation.
    group->setObject("EllipsoidModel", ellipsoidModel);

    return group;
}

vsg::ref_ptr<vsg::Object> TileReaderVOE::read_subtile(const osgEarth::TileKey& key,
                                                      vsg::ref_ptr<const vsg::Options> options) const
{
    // std::cout<<"Need to load subtile for "<<x<<", "<<y<<", "<<lod<<std::endl;

    // need to load subtile x y lod
    unsigned local_lod = key.getLOD() + 1;
    vsg::time_point start_read = vsg::clock::now();

    auto group = vsg::Group::create();

    std::vector<vsg::ref_ptr<vsg::Node>> childTiles;
    for (int i = 0; i < 4; ++i)
    {
        auto childKey = key.createChildKey(i);
        auto childTile = createTile(childKey, options);
        if (childTile && getTileStatus(childTile) != NoSuchTile)
        {
            childTile->setValue("tileName", childKey.str());
            childTiles.push_back(childTile);
        }
        else
            break;
    }

    if (childTiles.size()==4)
    {
        for(auto tile : childTiles)
        {
            vsg::ComputeBounds computeBound;
            tile->accept(computeBound);
            auto& bb = computeBound.bounds;
            vsg::dsphere newBound((bb.min.x + bb.max.x) * 0.5, (bb.min.y + bb.max.y) * 0.5, (bb.min.z + bb.max.z) * 0.5, vsg::length(bb.max - bb.min) * 0.5);

            if (local_lod < maxLevel)
            {
                auto plod = VoeLOD::create();
                plod->bound = newBound;
                plod->children[0] = VoeLOD::Child{lodTransitionScreenHeightRatio, {}}; // external child visible when it's bound occupies more than 1/4 of the height of the window
                plod->children[1] = VoeLOD::Child{0.0, tile}; // visible always
                std::string tileName;
                tile->getValue("tileName", tileName);
                plod->filename = vsg::make_string(tileName, ".tile");
                plod->options = options;

                //std::cout<<"plod->filename "<<plod->filename<<std::endl;

                group->addChild(plod);
            }
            else
            {
                auto cullGroup = vsg::CullGroup::create();
                cullGroup->bound = newBound;
                cullGroup->addChild(tile);

                group->addChild(cullGroup);
            }


        }
    }

    vsg::time_point end_read = vsg::clock::now();

    double time_to_read_tile =  std::chrono::duration<float, std::chrono::milliseconds::period>(end_read - start_read).count();

    {
        std::scoped_lock<std::mutex> lock(statsMutex);
        numTilesRead += 1;
        totalTimeReadingTiles += time_to_read_tile;
    }

    if (group->children.size() != 4)
    {
        OE_DEBUG << "Warning: could not load all 4 subtiles, loaded only " << group->children.size() << std::endl;
        setTileStatus(group, NoSuchTile);
    }

    return group;
}

void TileReaderVOE::init(vsg::CommandLine& commandLine,  vsg::ref_ptr<const vsg::Options> options)
{
    osg::ArgumentParser parser = convertArgs(commandLine);
    if (!(mapNode  = osgEarth::MapNode::load(parser)))
    {
        throw std::runtime_error("no map");
    }
    osgEarth::Map* map = mapNode->getMap();
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
    if (!getElevations())
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

vsg::ref_ptr<vsg::StateGroup> TileReaderVOE::createRoot() const
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
    if (getReverseDepth())
    {
        vsg::ShaderStage::SpecializationConstants specializationConstants{
        {0, vsg::uintValue::create(1)}
        };
        vertexShader->specializationConstants = specializationConstants;
        depthStencilState->depthCompareOp = VK_COMPARE_OP_GREATER;
    }
    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::RasterizationState::create(),
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        depthStencilState};

    auto graphicsPipeline = vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    auto root = vsg::StateGroup::create();
    auto lightDescriptorSet = vsg::DescriptorSet::create(simState.light_descriptorSetLayout,
                                                         vsg::Descriptors{simState.lightValues});
    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, lightDescriptorSet);
    bindDescriptorSet->slot = 2;
    root->add(bindGraphicsPipeline);
    root->add(bindDescriptorSet);
    return root;
}

// unpleasant workaround for vsgXchange

vsg::ref_ptr<vsg::Node>
TileReaderVOE::createTile(const osgEarth::TileKey& key, vsg::ref_ptr<const vsg::Options> options) const
{
    osgEarth::GeoImage gimage = imageLayer->createImage(key);
    osg::ref_ptr<osgEarth::ElevationTexture> elevationTexture;
    // create StateGroup to bind any texture state
    auto scenegraph = vsg::StateGroup::create();

    if (!gimage.valid())
    {
        setTileStatus(scenegraph, NoSuchTile);
        return scenegraph;
    }
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
    setTileStatus(scenegraph, Valid);
    return scenegraph;
}
