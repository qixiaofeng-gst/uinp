#version 460 core
#extension GL_ARB_shading_language_include : require
#include "complex"
#include "colormap"

in vec2 uv;

out vec3 color;

uniform float uniform_time;
uniform sampler2D texture_sampler;

/**
Play with Mandelbrot set: z_{n+1} = z_n^2 + c, z_0 = 0 + 0i
The 'c' is the given pixel position, render the iteration count that make z_n > 2
*/

vec3 draw_mandelbrot(vec2 point, float passed_time) {
    // vec2 c = point / passed_time - vec2(0.1, 0.25);
    vec2 c = point / (passed_time * 1e3) + vec2(0.3151, 0.5589);
    vec2 p = mod(c, 0.25);
    if (length(p) < 0.01) {
        return vec3(1.0);
    }
    vec2 z = vec2(0);
    float max_iteration = 1024.0;
    for (int i = 0; i < max_iteration; ++i) {
        z = complex_power(z, 2) + c;
        if (length(z) > 2) {
            return get_color(float(i) / max_iteration);
        }
    }
    return vec3(0.0);
}

void main() {
  color = draw_mandelbrot(uv, uniform_time);
}
