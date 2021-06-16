#version 460 core

smooth in vec3 smooth_color;
in vec2 uv;

out vec3 color;

uniform sampler2D texture_sampler;

void main() {
  color.xyz = smooth_color;
  color = texture(texture_sampler, uv).rgb;
}
