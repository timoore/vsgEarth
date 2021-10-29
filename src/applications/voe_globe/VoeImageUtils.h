#pragma once

// Wrapper around vsgXchange functions

#include <vsg/all.h>
#include <osg/Image>

namespace voe
{
    vsg::ref_ptr<vsg::Data> convertToVsg(const osg::Image* image, bool mapRGBtoRGBAHint = false);
}
