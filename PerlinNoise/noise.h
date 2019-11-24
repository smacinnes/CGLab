#pragma once

#include <cstdlib>
#include "OpenGP/GL/Application.h"

using namespace OpenGP;

inline float lerp(float x, float y, float t) {
    /// TODO: Implement linear interpolation between x and y
    return x+t*(y-x);
}

inline float fade(float t) {
    return t * t * t * (t * (t * 6 - 15) + 10);
}

inline float rand01() {
    return float(std::rand())/float(RAND_MAX);
}

float* perlin2D(const int width, const int height, const int period=64);

/// Create a heightmap directly from perlin noise (as opposed to using fBM as in assignment 3)
R32FTexture* perlin2DTexture() {

    ///--- Precompute perlin noise on a 2D grid
    const int width = 512;
    const int height = 512;
    float *perlin_data = perlin2D(width, height, 128);

    R32FTexture* _tex = new R32FTexture();
    _tex->upload_raw(width, height, perlin_data);
    return _tex;
}

/// Generates a heightmap using fractional brownian motion
// R32FTexture* fBm2DTexture() {} // Ommited, part of assignment 3

float* perlin2D(const int width, const int height, const int period) {

    ///--- Precompute random gradients
    float *gradients = new float[width*height*2];
    auto sample_gradient = [&](int i, int j) {
        float x = gradients[2*(i+j*height)];
        float y = gradients[2*(i+j*height)+1];
        return Vec2(x,y);
    };

    for (int i = 0; i < width; ++ i) {
        for (int j = 0; j < height; ++ j) {
            float angle = rand01();
            gradients[2*(i+j*height)] = cos(2 * angle * float(M_PI));
            gradients[2*(i+j*height)+1] = sin(2 * angle * float(M_PI));
        }
    }

    ///--- Perlin Noise parameters
    float frequency = 1.0f / period;

    float *perlin_data = new float[width*height];
    for (int i = 0; i < width; ++ i) {
        for (int j = 0; j < height; ++ j) {

            ///--- Integer coordinates of corners
            int left = (i / period) * period;
            int right = (left + period) % width;
            int top = (j / period) * period;
            int bottom = (top + period) % height;

            ///--- local coordinates [0,1] within each block
            float dx = (i - left) * frequency;
            float dy = (j - top) * frequency;

            ///--- Fetch random vectors at corners
            Vec2 topleft = sample_gradient(left, top);
            Vec2 topright = sample_gradient(right, top);
            Vec2 bottomleft = sample_gradient(left, bottom);
            Vec2 bottomright = sample_gradient(right, bottom);

            ///--- Vector from each corner to pixel center
            Vec2 a(dx,      -dy); // topleft
            Vec2 b(dx-1,    -dy); // topright
            Vec2 c(dx,      1 - dy); // bottomleft
            Vec2 d(dx-1,    1 - dy); // bottomright

            ///TODO: Get scalars at corners HINT: take dot product of gradient and corresponding direction
            float s = topleft.dot(a);
            float t = topright.dot(b);
            float u = bottomleft.dot(c);
            float v = bottomright.dot(d);

            ///TODO: Interpolate along "x" HINT: use fade(dx) as t
            float st = lerp(s,t,fade(dx));
            float uv = lerp(u,v,fade(dx));

            ///TODO: Interpolate along "y"
            float noise = lerp(st,uv,fade(dy));

            perlin_data[i+j*height] = noise;
        }
    }

    delete[] gradients;
    return perlin_data;
}
