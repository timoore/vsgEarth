#pragma once

#include <vsg/all.h>
#include <osgEarth/VisibleLayer>
#include <osgEarth/TerrainTileModelFactory>
#include <vsg/core/Array.h>
#include <vsg/core/Inherit.h>
#include <vsg/core/Object.h>
#include <vsg/core/ref_ptr.h>
#include <vsg/state/Descriptor.h>
#include <vsg/state/DescriptorBuffer.h>

// Class responsible for assembling a tile from the osgEarth map. In practice this means that it is
// responsible for setting up the StateGroup nodes that hold the BindGraphicsPipeline objects that
// are used to render the tiles. Management of the vsg::PagedLOD structure is delegated to
// TileReaderVOE.

#include "Export.h"
#include "TileReaderVOE.h"
#include "VoeSim.h"

namespace osgEarth
{
    struct TileParams
    {
        TileParams(int numLayers)
            : numLayers(numLayers)
        {
            data = vsg::vec4Array::create(4 * numLayers + 4 + 4 + 1);
        }
        int numLayers;
        vsg::mat4& imageTexMatrix(int i)
        {
            return *reinterpret_cast<vsg::mat4*>(&(*data)[i * 4]);
        }
        vsg::mat4& elevationTexMatrix()
        {
            return *reinterpret_cast<vsg::mat4*>(&(*data)[numLayers * 4]);
        }

        vsg::mat4& normalTexMatrix()
        {
            return *reinterpret_cast<vsg::mat4*>(&(*data)[(numLayers + 1) * 4]);
        }

        vsg::vec2& elevTexelCoeff()
        {
            return *reinterpret_cast<vsg::vec2*>(&(*data)[(numLayers + 2) * 4]);
        }

        vsg::ref_ptr<vsg::vec4Array> data;
    };

    struct LayerParams : public vsg::Inherit<vsg::Object, LayerParams>
    {
        LayerParams(int numLayers)
            : numLayers(numLayers)
        {
            data = vsg::vec4Array::create(numLayers);
            layerParamsDescriptor = vsg::DescriptorBuffer::create(data, 1);
        }
        int numLayers;
        
        bool enabled(int layer) const
        {
            return (*data)[layer][0] == 1.0f;
        }

        void setEnabled(int layer, bool val)
        {
            (*data)[layer][0] = val;
        }

        float opacity(int layer) const
        {
            return (*data)[layer][1];
        }

        void setOpacity(int layer, float opacity)
        {
            (*data)[layer][1] = opacity;
        }

        ColorBlending blendMode(int layer) const
        {
            return (*data)[layer][2] == 0.0 ? BLEND_INTERPOLATE : BLEND_MODULATE;
        }

        void setBlendMode(int layer, ColorBlending blendMode)
        {
            (*data)[layer][2] = blendMode;
        }
        
        vsg::ref_ptr<vsg::vec4Array> data;
        vsg::ref_ptr<vsg::DescriptorBuffer> layerParamsDescriptor;
    };

    struct VOELayerCallback : public VisibleLayerCallback
    {
        VOELayerCallback(vsg::ref_ptr<LayerParams> layerParams, int layerIdx)
            : layerParams(layerParams), layerIdx(layerIdx)
        {
        }
        vsg::ref_ptr<LayerParams> layerParams;
        void onVisibleChanged(class VisibleLayer* layer) override;
        void onOpacityChanged(class VisibleLayer* layer) override;

        const int layerIdx;
    };
    
    class VSGEARTH_EXPORT TerrainEngineVOE : public vsg::Inherit<vsg::Object, TerrainEngineVOE>
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
        osgEarth::MapNode* init(vsg::ref_ptr<vsg::Options> options, vsg::CommandLine& arguments);
        vsg::ref_ptr<vsg::Node> createScene(vsg::ref_ptr<vsg::Options> options);
        vsg::ref_ptr<vsg::Node> createTile(const osgEarth::TileKey& key,
                                           vsg::ref_ptr<const vsg::Options> options) const;
        void update(vsg::ref_ptr<vsg::Viewer> viewer, vsg::ref_ptr<vsg::Camera> camera);
        osgEarth::Map* getMap() { return mapNode->getMap(); }
        const osgEarth::Map* getMap() const { return mapNode->getMap(); }
        bool getReverseDepth() const { return reverseDepth; }
        void setReverseDepth(bool val) { reverseDepth = val; }
        bool getElevations() const { return elevations; }
        void setElevations(bool val) { elevations = val; }
        vsg::ref_ptr<WireframeInputHandler> createWireframeHandler();
        vsg::ref_ptr<TileReaderVOE> tileReader;
        std::string projection;
        vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel;

        osg::ref_ptr<osgEarth::MapNode> mapNode;
        SimpleLight simState;
        uint32_t mipmapLevelsHint;
        uint32_t numImageLayers;
    protected:
        void initLayers();
        vsg::ref_ptr<vsg::DescriptorSetLayout> descriptorSetLayout;
        vsg::ref_ptr<vsg::DescriptorSetLayout> lightsDescriptorSetLayout;
        vsg::ref_ptr<vsg::DescriptorSetLayout> layerDescriptorSetLayout;
        vsg::ref_ptr<vsg::PipelineLayout> pipelineLayout;
        vsg::ref_ptr<vsg::Sampler> sampler;
        vsg::ref_ptr<vsg::Sampler> elevationSampler;
        vsg::ref_ptr<vsg::Sampler> normalSampler;
        vsg::ref_ptr<vsg::Switch> sceneRootSwitch;
        bool reverseDepth = true;
        bool elevations = true;
        osg::ref_ptr<osgEarth::TerrainTileModelFactory> modelFactory;
        vsg::ref_ptr<LayerParams> layerParams;
    };
}
