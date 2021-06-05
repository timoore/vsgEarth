#ifndef VOESDK_H
#define VOESDK_H 1

layout(binding = 0) uniform sampler2D texSampler; // only available in fragment shader
layout(binding = 1) uniform sampler2D elevationTex;
layout(binding = 2) uniform sampler2D normalTex;

layout(set = 0, binding = 3) uniform VOETile {
    mat4 elevationTexMatrix;
    mat4 normalTexMatrix;
    vec2 elevTexelCoeff;
} voeTile;

float oe_terrain_getElevation(in vec2 uv);
vec4 oe_terrain_getNormalAndCurvatureScaled(in vec2 uv_scaledBiased);
vec4 oe_terrain_getNormalAndCurvature(in vec2 st);

#endif
