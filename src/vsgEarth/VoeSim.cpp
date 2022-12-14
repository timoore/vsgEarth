#include "VoeSim.h"

using namespace osgEarth;

// Some "sensible" defaults; direction from sun over the Atlantic Ocean
SimpleLight::SimpleLight()
    : worldDirection(-0.7071067811865476, 0.7071067811865476, 0.0)
{
    _value = SimpleLightUniformValue::create();
    // Eye-space direction just to have something initialized, but
    // should be updated with the view matrix.
    setDirection(vsg::vec3(worldDirection.x, worldDirection.y, worldDirection.z));
    setColor(vsg::vec3(.8f, .8f, .8f));
    setAmbient(vsg::vec3(.2f, .2f, .2f));
    lightValues = vsg::DescriptorBuffer::create(_value, 0);
}

void SimpleLight::setEyeDirection(const vsg::dmat4& viewMatrix)
{
    vsg::dvec4 dLightDirectionWorld(worldDirection.x, worldDirection.y, worldDirection.z, 0.0);
    vsg::dvec4 dLightDirectionEye = viewMatrix * dLightDirectionWorld;
    vsg::vec3 lightDirectionEye(dLightDirectionEye.x, dLightDirectionEye.y, dLightDirectionEye.z);
    setDirection(lightDirectionEye);
}
