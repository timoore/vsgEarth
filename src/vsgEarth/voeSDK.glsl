
#include "VoeSDK.h"

// Utility functions that should work in any stage

/**
 * Sample the elevation data at a UV tile coordinate.
 */
float oe_terrain_getElevation(in vec2 uv)
{
    // Texel-level scale and bias allow us to sample the elevation texture
    // on texel center instead of edge.
    vec2 elevc = uv
        * voeTile.elevTexelCoeff.x * voeTile.elevationTexMatrix[0][0]     // scale
        + voeTile.elevTexelCoeff.x * voeTile.elevationTexMatrix[3].st     // bias
        + voeTile.elevTexelCoeff.y;

    return texture(elevationTex, elevc).r;
}

/**
 * Read the normal vector and curvature at resolved UV tile coordinates.
 */
vec4 oe_terrain_getNormalAndCurvatureScaled(in vec2 uv_scaledBiased)
{
    vec4 n = texture(normalTex, uv_scaledBiased);
    n.xyz = n.xyz*2.0-1.0;
    float curv = n.z;
    n.z = 1.0 - abs(n.x) - abs(n.y);
    // unnecessary since Z is never < 0:
    //float t = clamp(-n.z, 0, 1);
    //n.x += (n.x > 0)? -t : t;
    //n.y += (n.y > 0)? -t : t;
    return vec4(normalize(n.xyz), curv);
}

vec4 oe_terrain_getNormalAndCurvature(in vec2 st)
{
    vec2 uv_scaledBiased = st
        * voeTile.elevTexelCoeff.x * voeTile.normalTexMatrix[0][0]
        + voeTile.elevTexelCoeff.x * voeTile.normalTexMatrix[3].st
        + voeTile.elevTexelCoeff.y;

    return oe_terrain_getNormalAndCurvatureScaled(uv_scaledBiased);
}

bool oe_layerEnabled(int layer)
{
    return voeLayers.imageLayerParams[layer].x == 0.0 ? false : true;
}

float oe_layerOpacity(int layer)
{
    return voeLayers.imageLayerParams[layer].y;
}

int oe_layerColorBlending(int layer)
{
    return voeLayers.imageLayerParams[layer].z == 0.0 ? 0 : 1;
}

vec4 oe_blendLayerColor(vec4 color, int layer)
{
    vec4 layerColor;
    float oe_layer_opacity = oe_layerOpacity(layer);
    if (oe_layerColorBlending(layer) == OE_BLEND_INTERPOLATE)
    {
        layerColor.rgb = color.xyz;
        layerColor.a = color.a * oe_layer_opacity;
    }
    else
    {
        const float OE_MODULATION_EXPOSURE = 2.35;
        vec3 rgbHi = (oe_layer_opacity > 0.0
                      ? color.rgb * OE_MODULATION_EXPOSURE / oe_layer_opacity
                      : vec3(1.0));
        layerColor.rgb = mix(vec3(1.0), rgbHi, oe_layer_opacity);
        layerColor.a = 1.0;
    }
    return layerColor;
}

// Fake entry point XXX Still needed, or for linking experiments that didn't work?

void voeSDKFoo()
{}
