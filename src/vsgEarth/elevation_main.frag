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
    vec3 vp_Normal = normalize( oe_normalMapTBN*normalAndCurvature.rgb );
    vec3 color = vec3(0.0, 0.0, 0.0);
    vec4 lightNums = lightData.values[0];
    int numAmbientLights = int(lightNums[0]);
    int numDirectionalLights = int(lightNums[1]);
    int numPointLights = int(lightNums[2]);
    int numSpotLights = int(lightNums[3]);
    int index = 1;
    outColor = vec4(1.0, 1.0, 1.0, 1.0);

    vec4 diffuseColor = vec4(0.0, 0.0, 0.0, 0.0);
    for (int i = 0; i < voeTile.numImageLayers; ++i)
    {
        if (oe_layerEnabled(i))
        {
            vec4 baseColor = texture(texSampler[i], fragTexCoord) * vec4(fragColor.rgb, 1.0);
            vec4 layerColor = oe_blendLayerColor(baseColor, i);
            // formulae for blending with destination alpha
            diffuseColor.a = layerColor.a + diffuseColor.a * (1 - layerColor.a);
            diffuseColor.rgb
                = (layerColor.rgb * layerColor.a + diffuseColor.rgb * diffuseColor.a * (1.0 - layerColor.a)) / diffuseColor.a;
        }
    }

    if (numAmbientLights>0)
    {
        // ambient lights
        for(int i = 0; i<numAmbientLights; ++i)
        {
            vec4 ambient_color = lightData.values[index++];
            color += diffuseColor.rgb * ambient_color.rgb * ambient_color.a;
        }
    }

    if (numDirectionalLights>0)
    {
        // directional lights
        for(int i = 0; i<numDirectionalLights; ++i)
        {
            vec4 lightColor = lightData.values[index++];
            vec3 direction = -lightData.values[index++].xyz;
            float diff = max(dot(direction, vp_Normal), 0.0);
            color.rgb += (diffuseColor.rgb * lightColor.rgb) * (diff * lightColor.a);
            // Not doing specular yet
#if 0
            if (diff > 0.0)
            {
                vec3 halfDir = normalize(direction + vd);
                color.rgb += specularColor.rgb * (pow(max(dot(halfDir, nd), 0.0), shininess) * lightColor.a);
            }
#endif
        }
    }
    outColor.rgb = color.rgb;
}
