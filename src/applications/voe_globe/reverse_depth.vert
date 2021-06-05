#version 450

#include "VoeSDK.h"

layout(constant_id = 0) const uint reverseDepth = 0;

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inEllipsoidNormal;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragTexCoord;

out gl_PerVertex {
    vec4 gl_Position;
};

void main() {
    mat4 projection = pc.projection;

    if (reverseDepth != 0)
    {
        projection[2][2] *= -pc.projection[2][2] - 1.0;
        projection[3][2] *= -1.0;
    }
    float elevation = oe_terrain_getElevation(inTexCoord);
    vec3 position = inPosition + inEllipsoidNormal * elevation;
    gl_Position = (projection * pc.modelview) * vec4(position, 1.0);
    fragColor = inColor;
    fragTexCoord = inTexCoord;
}