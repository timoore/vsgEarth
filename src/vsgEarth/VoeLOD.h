#pragma once

// A better behaved PagedLOD that deals with missing tiles and such.

#include <vsg/nodes/PagedLOD.h>

namespace voe
{
    enum TileStatus
    {
        Valid,
        NoSuchTile,
        Error
    };

    TileStatus getTileStatus(const vsg::Node* node);
    void setTileStatus(vsg::Node* node, TileStatus status);
    
    class VoeLOD : public vsg::Inherit<vsg::PagedLOD, VoeLOD>
    {
    public:
        VoeLOD(vsg::Allocator* allocator = nullptr)
            : Inherit(allocator)
        {}
        void accept(vsg::RecordTraversal& visitor) const override;
    };
}

