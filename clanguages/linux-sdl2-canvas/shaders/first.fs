#version 460 core
#extension GL_ARB_shading_language_include : require
#include "complex"
#include "colormap"

in vec2 uv;

out vec3 color;

uniform float uniform_time;
uniform sampler2D texture_sampler;

vec2 f_for_newton_fractal(vec2 z) {
  return complex_power(z, 3) - vec2(1, 0);
}

vec2 fd_for_newton_fractal(vec2 z) {
  return 3.0 * complex_power(z, 2);
}

const vec2 roots_p3[] = vec2[](
  vec2(1, 0),
  vec2(-0.5, 0.866),
  vec2(-0.5, -0.866)
);

const vec3 colors[] = vec3[](
  vec3(1.0, 0.8, 0.0),
  vec3(0.0, 1.0, 0.8),
  vec3(0.8, 0.0, 1.0)
);

const int roots_count = roots_p3.length();

vec3 draw_newton_fractal_p3_scaling(vec2 pixel_position, float time) {
    float a = (sin(time * 0.1) + 1.0) * 0.99 + 0.01;
    vec2 z = pixel_position * 5 - 2.5;
    for (int i = 0; i < 1024; ++i) {
        z -= complex_divide(f_for_newton_fractal(z), fd_for_newton_fractal(z)) * a;
        for (int j = 0; j < roots_count; ++j) {
            vec2 d = z - roots_p3[j];
            if (length(d) < 1e-5) {
                return colors[j];
            }
        }
    }
    return vec3(0.0);
}

/**
Play with Mandelbrot set: z_{n+1} = z_n^2 + c, z_0 = 0 + 0i
The 'c' is the given pixel position, render the iteration count that make z_n > 2
*/

/**
TODO: Play with Julia set.
*/

void main() {
  color = draw_newton_fractal_p3_scaling(uv, uniform_time);
}
