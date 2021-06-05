#pragma once

// Class that defines attributes driven by the voe "simulation,"
// e.g. lighting.
#include <vsg/all.h>

namespace voe
{
    // The data structure passed to shaders. These values are mostly
    // vec3, but are declared vec4 to maintain alignment.
    
    struct SimpleLightUniform
    {
        vsg::vec4 direction;
        vsg::vec4 color;
        vsg::vec4 ambient;
    };

    struct SimpleLightUniformValue :
        public vsg::Inherit<vsg::Value<SimpleLightUniform>, SimpleLightUniformValue>
    {
    };
    
    class SimpleLight
    {
    public:
        vsg::ref_ptr<vsg::DescriptorBuffer> lightValues;
        vsg::ref_ptr<vsg::DescriptorSetLayout> light_descriptorSetLayout;
        SimpleLight();

        vsg::ref_ptr<SimpleLightUniformValue>& value() { return _value; }

        // Eye direction
        void setDirection(const vsg::vec3& direction)
        {
            _value->value().direction = vsg::vec4(direction.x, direction.y, direction.z, 0.0f);
        }

        const vsg::vec3& getDirection() const
        {
            return *reinterpret_cast<const vsg::vec3*>(&_value->value().direction);
        }
        
        void setColor(const vsg::vec3& color)
        {
            _value->value().color = vsg::vec4(color.x, color.y, color.z, 1.0f);
        }

        const vsg::vec3& getColor() const
        {
            return *reinterpret_cast<const vsg::vec3*>(&_value->value().color);
        }

        void setAmbient(const vsg::vec3& color)
        {
            _value->value().ambient = vsg::vec4(color.x, color.y, color.z, 1.0f);
        }

        const vsg::vec3& getAmbient() const
        {
            return *reinterpret_cast<const vsg::vec3*>(&_value->value().ambient);
        }

        vsg::dvec3 worldDirection;
    protected:
        vsg::ref_ptr<SimpleLightUniformValue> _value;
        
    };
}
