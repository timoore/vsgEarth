#pragma once

// A better behaved PagedLOD that deals with missing tiles and such.

#include <vsg/nodes/PagedLOD.h>

namespace osgEarth
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
        VoeLOD()
        {}
        void accept(vsg::RecordTraversal& visitor) const override;
    };
}

