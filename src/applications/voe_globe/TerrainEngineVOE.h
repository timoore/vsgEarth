#pragma once

#include <vsg/all.h>

// Class responsible for assembling a tile from the osgEarth map. In practice this means that it is
// responsible for setting up the StateGroup nodes that hold the BindGraphicsPipeline objects that
// are used to render the tiles. Management of the vsg::PagedLOD structure is delegated to
// TileReaderVOE.

#include "TileReaderVOE.h"
#include "VoeSim.h"

namespace voe
{
    class TerrainEngineVOE : public vsg::Inherit<vsg::Object, TerrainEngineVOE>
    {
    public:
        class WireframeInputHandler : public vsg::Inherit<vsg::Visitor, WireframeInputHandler>
        {
            vsg::observer_ptr<vsg::Switch> switchNode;
            unsigned state;
        public:
            WireframeInputHandler(vsg::ref_ptr<vsg::Switch>& switchNode);
            void apply(vsg::KeyPressEvent& keyPress) override;
        };

        TerrainEngineVOE();
        void init(vsg::ref_ptr<vsg::Options> options, vsg::CommandLine& arguments);
        vsg::ref_ptr<vsg::Node> createScene(vsg::ref_ptr<vsg::Options> options);
        vsg::ref_ptr<vsg::Node> createTile(const osgEarth::TileKey& key,
                                           vsg::ref_ptr<const vsg::Options> options) const;
        osgEarth::Map* getMap() { return mapNode->getMap(); }
        vsg::ref_ptr<WireframeInputHandler> createWireframeHandler();
        vsg::ref_ptr<TileReaderVOE> tileReader;
        std::string projection;
        vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel;

        osg::ref_ptr<osgEarth::MapNode> mapNode;
        SimpleLight simState;
        uint32_t mipmapLevelsHint;
    protected:
        vsg::ref_ptr<vsg::DescriptorSetLayout> descriptorSetLayout;
        vsg::ref_ptr<vsg::PipelineLayout> pipelineLayout;
        vsg::ref_ptr<vsg::Sampler> sampler;
        vsg::ref_ptr<vsg::Sampler> elevationSampler;
        vsg::ref_ptr<vsg::Sampler> normalSampler;
        osg::ref_ptr<osgEarth::ImageLayer> imageLayer;
        vsg::ref_ptr<vsg::Switch> sceneRootSwitch;
        bool reverseDepth = true;
        bool elevations = true;

    };
}