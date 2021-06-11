#version 460 core

out vec3 color;

smooth in vec3 smooth_color;

void main() {
  color.xyz = smooth_color;
}
