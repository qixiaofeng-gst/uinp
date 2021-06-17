#version 460 core

in vec2 uv;

out vec3 color;

uniform sampler2D texture_sampler;

vec3 draw_quater_circle(vec2 pixel_position) {
  if (length(pixel_position) > 0.5) {
    return vec3(pixel_position, 0.5);
  } else {
    return vec3(0.3, pixel_position * 1.5);
  }
}

void main() {
  // color = texture(texture_sampler, uv).rgb;
  // color = draw_quater_circle(uv);

  // https://www.shadertoy.com/view/7ljGzR (7, towers)
  color = vec3(0, 0, 0);
  ivec3 b = ivec3(127, 128, 255);
  for(;(b.x ^ b.y & b.z) % 200 > (b.z - 9);) {
    b = ivec3((uv / 5e2 - 0.5) * color.x + 5.0 /*iTime*/ /.1, color += .1);
  }
  color = vec3(b * b.x % 2) + color / 2e2;
}
