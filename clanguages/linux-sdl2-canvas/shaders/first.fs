#version 460 core

in vec2 uv;

out vec3 color;

uniform float uniform_time;
uniform sampler2D texture_sampler;

const float gray_scale = 1e-2;
const float cycle_size = 1.0;
const vec2 cycle_origin = vec2(0.5);

vec3 draw_quater_circle(vec2 pixel_position) {
  if (length(pixel_position) > 0.5) {
    return vec3(pixel_position, 0.5);
  } else {
    return vec3(0.3, pixel_position * 1.5);
  }
}

vec3 draw_towers_7ljGzR(vec2 pixel_position, vec3 old_color, float time) {
  ivec3 b = ivec3(127, 128, 255);
  for(;(b.x ^ b.y & b.z) % 200 > (b.z - 9);) {
    b = ivec3((pixel_position / 5e2 - 0.5) * old_color.x + time /.1, old_color += .1);
  }
  return vec3(b * b.x % 2) + old_color / 2e2;
}

vec3 draw_shining_XsXXDn(vec2 pixel_position, float time) {
  float x = mod(time, cycle_size) - cycle_size * 0.5;
  float y = x * x;
  return vec3(gray_scale) * y / length(pixel_position - cycle_origin);
}

vec3 draw_halo(vec2 pixel_position, float time) {
  float x = mod(time, cycle_size) - cycle_size * 0.5;
  float y = x * x;
  return vec3(sin(length(pixel_position - cycle_origin) * 1e2 * y));
}

vec3 draw_parabola(vec2 pixel_position) {
  if (abs((pixel_position.x * pixel_position.x) - pixel_position.y) < 1e-2) {
    return vec3(0.0);
  }
  return vec3(1.0);
}

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

vec3 draw_newton_fractal(vec2 pixel_position) {
    vec2 z = pixel_position;
    for (int i = 0; i < 100; ++i) {
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

// https://www.shadertoy.com/view/______
void main() {
  // color = vec3(1.0, 0, mod(uniform_time, 1.0));
  // color = texture(texture_sampler, uv).rgb;
  // color = draw_quater_circle(uv);
  // color = draw_towers_7ljGzR(uv, vec3(0.0, 0.0, 0.0), uniform_time);
  // color = draw_shining_XsXXDn(uv, uniform_time);
  // color = draw_halo(uv, uniform_time);
  // color = draw_parabola(uv);
  color = draw_newton_fractal(uv);
}
