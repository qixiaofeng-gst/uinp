#version 460 core
#extension GL_ARB_shading_language_include : require
//https://stackoverflow.com/questions/10754437/how-to-using-the-include-in-glsl-support-arb-shading-language-include
//#include "/home/qixiaofeng/Documents/git-repos/uinp/clanguages/linux-sdl2-canvas/shaders/complex.gl"

in vec2 uv;

out vec3 color;

uniform float uniform_time;
uniform sampler2D texture_sampler;

vec2 complex_multiply(vec2 a, vec2 b) {
    return vec2(a.x * b.x - a.y * b.y, a.y * b.x + a.x * b.y);
}

vec2 complex_power(vec2 z, int p) {
    vec2 result = z;
    for (int i = 0; i < (p - 1); ++i) {
        result = complex_multiply(result, z);
    }
    return result;
}

vec2 complex_divide(vec2 a, vec2 b) {
    vec2 b2 = b * b;
    float c = 1.0 / (b2.x + b2.y);
    return vec2(a.x * b.x + a.y * b.y, a.y * b.x - a.x * b.y) * c;
}

vec2 f_for_newton_fractal(vec2 z) {
  return complex_power(z, 3) - vec2(1, 0);
}

vec2 fd_for_newton_fractal(vec2 z) {
  return 3.0 * complex_power(z, 2);
}

const vec2 roots[3] = vec2[3](
  vec2(1, 0),
  vec2(-0.5, 0.866),
  vec2(-0.5, -0.866)
);

const vec3 colors[3] = vec3[3](
  vec3(1.0, 0.8, 0.0),
  vec3(0.0, 1.0, 0.8),
  vec3(0.8, 0.0, 1.0)
);

vec3 draw_newton_fractal(vec2 pixel_position, float time) {
    float c = (sin(time * 0.5) + 1.001) * 50.0;
    // float c = 1.0 / (time + 1e-3);
    vec2 z = pixel_position * c - vec2(c * 0.5);
    for (int i = 0; i < 1024; ++i) {
        z -= complex_divide(f_for_newton_fractal(z), fd_for_newton_fractal(z));
        for (int j = 0; j < 3; ++j) {
            vec2 d = z - roots[j];
            if (length(d) < 1e-5) {
                return colors[j];
            }
        }
    }
    return vec3(0.0);
}

void main() {
  color = draw_newton_fractal(uv, uniform_time);
}
