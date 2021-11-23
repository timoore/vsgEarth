#version 450

#include "VoeSDK.h"

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 oe_UpVectorView;
layout(location = 3) in vec2 oe_normalMapCoords;
layout(location = 4) in vec3 oe_normalMapBinormal;

layout(location = 0) out vec4 outColor;

void main()
{
    vec3 tangent = normalize(cross(oe_normalMapBinormal, oe_UpVectorView));
    mat3 oe_normalMapTBN = mat3(tangent, oe_normalMapBinormal, oe_UpVectorView);
    vec4 normalAndCurvature = oe_terrain_getNormalAndCurvature(oe_normalMapCoords);
    vec3 vp_Normal = normalize( oe_normalMapTBN*normalAndCurvature.xyz );
    vec3 illumination = voeLight.ambient.xyz;
    illumination += clamp(dot(vp_Normal, -voeLight.direction.xyz), 0.0, 1.0) * voeLight.color.xyz;
    outColor = vec4(0.0, 0.0, 0.0, 1.0);
    outColor.xyz = texture(texSampler[0], fragTexCoord).xyz * fragColor.xyz;
    outColor.xyz *= illumination;
}
