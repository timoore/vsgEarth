#pragma once

// Wrapper around vsgXchange functions

#include <vsg/all.h>
#include <osg/Image>

namespace voe
{
    vsg::ref_ptr<vsg::Data> convertToVsg(const osg::Image* image, bool mapRGBtoRGBAHint = false);

    template <typename T>
    struct VsgType;

    template<>
    struct VsgType<osg::Vec3f>
    {
        using type = vsg::vec3;
    };

    template<>
    struct VsgType<osg::Vec2f>
    {
        using type = vsg::vec2;
    };

    template<>
    struct VsgType<osg::Vec3d>
    {
        using type = vsg::dvec3;
    };

    template<>
    struct VsgType<osg::Vec2d>
    {
        using type = vsg::dvec2;
    };

    template <typename T>
    const typename VsgType<T>::type& toVsg(const T& val)
    {
        return *reinterpret_cast<const typename VsgType<T>::type*>(&val);
    }
}
