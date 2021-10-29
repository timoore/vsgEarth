#include "VoeImageUtils.h"

/* <editor-fold desc="MIT License">

Copyright(c) 2018 Robert Osfield

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

</editor-fold> */

// vsgXchange internals

namespace osg2vsg
{
    VkFormat convertGLImageFormatToVulkan(GLenum dataType, GLenum pixelFormat);

    osg::ref_ptr<osg::Image> formatImageToRGBA(const osg::Image* image);

    vsg::ref_ptr<vsg::Data> convertToVsg(const osg::Image* image, bool mapRGBtoRGBAHint);
}

namespace voe
{

    template<typename T>
    vsg::ref_ptr<vsg::Data> create(osg::ref_ptr<osg::Image> image, VkFormat format)
    {
        vsg::ref_ptr<vsg::Data> vsg_data;
        if (image->r() == 1)
        {
            vsg_data = vsg::Array2D<T>::create(image->s(), image->t(), reinterpret_cast<T*>(image->data()), vsg::Data::Layout{format});
        }
        else
        {
            vsg_data = vsg::Array3D<T>::create(image->s(), image->t(), image->r(), reinterpret_cast<T*>(image->data()), vsg::Data::Layout{format});
        }

        return vsg_data;
    }

    vsg::ref_ptr<vsg::Data> convertToVsg(const osg::Image* image, bool mapRGBtoRGBAHint)
    {
        if (!image || image->isCompressed() || image->getPixelFormat() != GL_RG)
            return osg2vsg::convertToVsg(image, mapRGBtoRGBAHint);

        osg::Image* new_image = const_cast<osg::Image*>(image);

        // we want to pass ownership of the new_image data onto th vsg_image so reset the allocation
        // mode on the image to prevent deletetion. 
        new_image->setAllocationMode(osg::Image::NO_DELETE);

        vsg::ref_ptr<vsg::Data> vsg_data;
        if (image->getDataType() == GL_UNSIGNED_BYTE)
            vsg_data = create<vsg::ubvec2>(new_image, VK_FORMAT_R8G8_UNORM);
        else if (image->getDataType() == GL_UNSIGNED_SHORT)
            vsg_data = create<vsg::usvec2>(new_image, VK_FORMAT_R16G16_UNORM);
        else if (image->getDataType() == GL_UNSIGNED_INT)
            vsg_data = create<vsg::uivec2>(new_image, VK_FORMAT_R32G32_UINT);
        else if (image->getDataType() == GL_FLOAT)
            vsg_data = create<vsg::vec2>(new_image, VK_FORMAT_R32G32_SFLOAT);
        else if (image->getDataType() == GL_DOUBLE)
            vsg_data = create<vsg::dvec2>(new_image, VK_FORMAT_R64G64_SFLOAT);
    
        vsg::Data::Layout& layout = vsg_data->getLayout();
        layout.maxNumMipmaps = image->getNumMipmapLevels();
        layout.origin = (image->getOrigin() == osg::Image::BOTTOM_LEFT) ? vsg::BOTTOM_LEFT : vsg::TOP_LEFT;

        return vsg_data;
    }
}
