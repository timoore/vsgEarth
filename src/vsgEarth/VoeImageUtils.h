#pragma once

// Wrapper around vsgXchange functions

#include <osg/Quat>
#include <vsg/all.h>
#include <osg/Image>
#include <vsg/maths/mat4.h>

namespace osgEarth
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

    template <>
    struct VsgType<osg::Matrixf>
    {
        using type = vsg::mat4;
    };

    template <>
    struct VsgType<osg::Matrixd>
    {
        using type = vsg::dmat4;
    };

    template <typename T>
    const typename VsgType<T>::type& toVsg(const T& val)
    {
        return *reinterpret_cast<const typename VsgType<T>::type*>(&val);
    }

    template <typename T>
    const typename VsgType<T>::type* toVsg(const T* val)
    {
        return reinterpret_cast<const typename VsgType<T>::type*>(val);
    }

    template <typename T>
    typename VsgType<T>::type* toVsg(T* val)
    {
        return reinterpret_cast<typename VsgType<T>::type*>(val);
    }
}
