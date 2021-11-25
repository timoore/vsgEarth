#include "TileReaderVOE.h"
#include "TerrainEngineVOE.h"
#include "VoeImageUtils.h"
#include "VoeLOD.h"

#include <osgEarth/Notify>
#include <stdexcept>

using namespace osgEarth;

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

TileReaderVOE::TileReaderVOE()
{
}

vsg::ref_ptr<vsg::Object> TileReaderVOE::read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options) const
{
    auto extension = vsg::fileExtension(filename);
    if (extension != "tile")
        return {};
    vsg::ref_ptr<TerrainEngineVOE> te(terrainEngine);
    if (!te)
        return {};

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
        osgEarth::TileKey key(lod, x, y, te->getMap()->getProfile());

        // std::cout<<"read("<<filename<<") -> tile_info = "<<tile_info<<", x = "<<x<<", y = "<<y<<", z = "<<lod<<std::endl;

        return read_subtile(key, options);
    }
}

vsg::ref_ptr<vsg::Object> TileReaderVOE::read_root(vsg::ref_ptr<const vsg::Options> options) const
{
    std::vector<osgEarth::TileKey> rootKeys;
    vsg::ref_ptr<TerrainEngineVOE> te(terrainEngine);
    if (!te)
        return {};
    te->getMap()->getProfile()->getRootKeys(rootKeys);
    auto sceneRoot = vsg::Group::create();
    for (const auto& key : rootKeys)
    {
        auto tile = te->createTile(key, options);
        if (tile)
        {
            vsg::ComputeBounds computeBound;
            tile->accept(computeBound);
            auto& bb = computeBound.bounds;
            vsg::dsphere newBound((bb.min.x + bb.max.x) * 0.5, (bb.min.y + bb.max.y) * 0.5, (bb.min.z + bb.max.z) * 0.5, vsg::length(bb.max - bb.min) * 0.5);

            auto plod = VoeLOD::create();
            plod->bound = newBound;
            plod->children[0] = VoeLOD::Child{lodTransitionScreenHeightRatio, {}}; 
            plod->children[1] = VoeLOD::Child{0.0, tile}; // visible always
            plod->filename = vsg::make_string(key.str(), ".tile");
            plod->options = options;

            sceneRoot->addChild(plod);
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
    vsg::CollectResourceRequirements collectRequirements;
    sceneRoot->accept(collectRequirements);
    sceneRoot->setObject("ResourceHints", collectRequirements.createResourceHints(tileMultiplier));

    return sceneRoot;
}

vsg::ref_ptr<vsg::Object> TileReaderVOE::read_subtile(const osgEarth::TileKey& key,
                                                      vsg::ref_ptr<const vsg::Options> options) const
{
    vsg::ref_ptr<TerrainEngineVOE> te(terrainEngine);
    if (!te)
        return {};

    // std::cout<<"Need to load subtile for "<<x<<", "<<y<<", "<<lod<<std::endl;

    // need to load subtile x y lod
    unsigned local_lod = key.getLOD() + 1;
    vsg::time_point start_read = vsg::clock::now();

    auto group = vsg::Group::create();

    std::vector<vsg::ref_ptr<vsg::Node>> childTiles;
    for (int i = 0; i < 4; ++i)
    {
        auto childKey = key.createChildKey(i);
        auto childTile = te->createTile(childKey, options);
        if (childTile)
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
    vsg::ref_ptr<TerrainEngineVOE> te(terrainEngine);
    if (!te)
        throw std::runtime_error("no terrain engine!");
    commandLine.read("-t", lodTransitionScreenHeightRatio);
    commandLine.read("-m", maxLevel);

}
