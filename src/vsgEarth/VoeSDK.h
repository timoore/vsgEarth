#ifndef VOESDK_H
#define VOESDK_H 1

layout(constant_id = 0) const uint reverseDepth = 0;
layout(constant_id = 1) const uint imageLayers = 1;



layout(binding = 0) uniform sampler2D texSampler[imageLayers]; // only available in fragment shader
layout(binding = 1) uniform sampler2D elevationTex;
layout(binding = 2) uniform sampler2D normalTex;

layout(set = 0, binding = 3) uniform VOETile {
    mat4 imageTexMatrix[imageLayers];
    mat4 elevationTexMatrix;
    mat4 normalTexMatrix;
    vec2 elevTexelCoeff;
} voeTile;

layout(set = 1, binding = 0) uniform VOELight {
    vec4 direction;
    vec4 color;
    vec4 ambient;
} voeLight;

float oe_terrain_getElevation(in vec2 uv);
vec4 oe_terrain_getNormalAndCurvatureScaled(in vec2 uv_scaledBiased);
vec4 oe_terrain_getNormalAndCurvature(in vec2 st);

#endif