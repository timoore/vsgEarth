#include <vsg/all.h>

#include "VoeLOD.h"

using namespace osgEarth;

TileStatus osgEarth::getTileStatus(const vsg::Node* node)
{
    TileStatus result = Valid;
    if (!node->getValue("voe_status", result))
    {
        return Valid;
    }
    else
    {
        return result;
    }
}

void osgEarth::setTileStatus(vsg::Node* node, TileStatus status)
{
    node->setValue("voe_status", status);
}

void VoeLOD::accept(vsg::RecordTraversal& visitor) const
{
    const auto& child = children[0];
    if (!child.node || getTileStatus(child.node) == Valid)
    {
        PagedLOD::accept(visitor);
        return;
    }
    // Simulate PagedLOD's traversal of child 1, which is actually
    // implemented in the RecordTraversal, yuck.
    auto &sphere = bound;

    // check if lod bounding sphere is in view frustum.
    if (!visitor.getState()->intersect(sphere))
    {
        // Punt let PagedLOD do its bookkeeping
        PagedLOD::accept(visitor);
        return;
    }

    const auto& proj = visitor.getState()->projectionMatrixStack.top();
    const auto& mv = visitor.getState()->modelviewMatrixStack.top();
    auto f = -proj[1][1];

    auto distance = std::abs(mv[0][2] * sphere.x + mv[1][2] * sphere.y + mv[2][2] * sphere.z + mv[3][2]);
    auto rf = sphere.r * f;

    // check the low res child to see if it's visible
    {
        const auto& loChild = children[1];
        auto cutoff = loChild.minimumScreenHeightRatio * distance;
        bool child_visible = rf > cutoff;
        if (child_visible)
        {
            if (loChild.node)
            {
                loChild.node->accept(visitor);
            }
        }
    }
}
