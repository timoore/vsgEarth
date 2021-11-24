#pragma once

#include <tuple>
#include <vsg/all.h>

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>

#include "VoeSim.h"

// Use osgEarth as a tile source

namespace voe
{
    class TerrainEngineVOE;
    struct TileParams
    {
        vsg::mat4 elevationTexMatrix;
        vsg::mat4 normalTexMatrix;
        vsg::vec2 elevTexelCoeff;
        vsg::vec2 padding;
    };

    struct TileParamsValue : public vsg::Inherit<vsg::Value<TileParams>, TileParamsValue>
    {
    };
    
    class TileReaderVOE : public vsg::Inherit<vsg::ReaderWriter, TileReaderVOE>
    {
        friend class TerrainEngineVOE;
    public:
        TileReaderVOE();
        // defaults for readymap / a globe
        vsg::dbox extents = {{-180.0, -90.0, 0.0}, {180.0, 90.0, 1.0}};
        uint32_t maxLevel = 22;
        bool originTopLeft = true;
        double lodTransitionScreenHeightRatio = 0.25;

        std::string projection;
        vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel;

        osg::ref_ptr<osgEarth::MapNode> mapNode;

        uint32_t mipmapLevelsHint = 16;

        void init(vsg::CommandLine& commandLine, vsg::ref_ptr<const vsg::Options> options = {});

        vsg::ref_ptr<vsg::Object> read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options = {}) const override;
        // timing stats
        mutable std::mutex statsMutex;
        mutable uint64_t numTilesRead{0};
        mutable double totalTimeReadingTiles{0.0};
        SimpleLight simState;
    protected:
        vsg::ref_ptr<vsg::Object> read_root(vsg::ref_ptr<const vsg::Options> options = {}) const;
        vsg::ref_ptr<vsg::Object> read_subtile(const osgEarth::TileKey& key, vsg::ref_ptr<const vsg::Options> options = {}) const;
        vsg::observer_ptr<TerrainEngineVOE> terrainEngine;
    };
}
