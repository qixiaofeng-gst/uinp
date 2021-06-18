#version 460 core

in vec2 uv;

out vec3 color;

uniform float uniform_time;
uniform sampler2D texture_sampler;

const float gray_scale = 1e-2;
const float cycle_size = 1.0;
const vec2 cycle_origin = vec2(0.3);

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

// https://www.shadertoy.com/view/______
void main() {
  // color = vec3(1.0, 0, mod(uniform_time, 1.0));
  color = draw_shining_XsXXDn(uv, uniform_time);
  // color = texture(texture_sampler, uv).rgb;
  // color = draw_quater_circle(uv);
  // color = draw_towers_7ljGzR(uv, vec3(0.0, 0.0, 0.0), uniform_time);
}
