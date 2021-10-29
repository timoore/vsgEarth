#include "VoeSim.h"

using namespace voe;

SimpleLight::SimpleLight()
    : worldDirection(0.0, -1.0, 0.0)
{
    _value = SimpleLightUniformValue::create();
    // Some "sensible" defaults
    setDirection(vsg::vec3(0.0f, -1.0f, 0.0f)); // This time an eye-space direction
    setColor(vsg::vec3(.8f, .8f, .8f));
    setAmbient(vsg::vec3(.2f, .2f, .2f));
    vsg::DescriptorSetLayoutBindings light_descriptorBindings{
        { 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }, 
        };
    lightValues = vsg::DescriptorBuffer::create(_value, 0);
    light_descriptorSetLayout = vsg::DescriptorSetLayout::create(light_descriptorBindings);
}
