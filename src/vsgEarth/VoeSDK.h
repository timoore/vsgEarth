// -*-GLSL-*-

#ifndef VOESDK_H
#define VOESDK_H 1

layout(constant_id = 0) const uint reverseDepth = 0;
layout(constant_id = 1) const uint maxImageLayers = 1;

layout(binding = 0) uniform sampler2D texSampler[maxImageLayers]; // only available in fragment shader
layout(binding = 1) uniform sampler2D elevationTex;
layout(binding = 2) uniform sampler2D normalTex;

struct TileImageLayer
{
    mat4 imageTexMatrix;
    uint layerIndex;            // Layer's index in voeLayers.imageLayerParams
    // 12 bytes padding
};

// This declaration will not work unless the variable array (imageLayers) is put at the
// end. Apparently the specialization constant cannot change the offsets of values in the structure,
// even though no error or warning is produced.
//
// https://stackoverflow.com/questions/52191104/specialization-constant-used-for-array-size
// describes the problem; although the context there is SPIRV in OpenGL, the issue is the same.

layout(set = 0, binding = 3) uniform VOETile {
    mat4 elevationTexMatrix;
    vec4 elevTexelCoeff;
    uint numImageLayers;          // Number of image layers in this tile
    // 12 bytes padding
    TileImageLayer imageLayers[maxImageLayers];
} voeTile;

layout(set = 1, binding = 0) uniform VOELight {
    vec4 direction;
    vec4 color;
    vec4 ambient;
} voeLight;

// Wasteful, but preserves padding:
// imageLayerParams[0] - enabled
// imageLayerParams[1] - opacity
// imageLayerParams[2] - blend mode: 0 - BLEND_INTERPOLATE, 1 - BLEND_MODULATE
layout(set = 2, binding = 0) uniform VOELayers {
    vec4 imageLayerParams[maxImageLayers];
} voeLayers;

#define OE_BLEND_INTERPOLATE 0
#define OE_BLEND_MODULATE 1

// layer parameter is index into the tile's layers i.e. the list in voeTile
bool oe_layerEnabled(int layer);
float oe_layerOpacity(int layer);
int oe_layerColorBlending(int layer);
vec4 oe_blendLayerColor(vec4 color, int layer);

float oe_terrain_getElevation(in vec2 uv);
vec4 oe_terrain_getNormalAndCurvatureScaled(in vec2 uv_scaledBiased);
vec4 oe_terrain_getNormalAndCurvature(in vec2 st);

#endif
