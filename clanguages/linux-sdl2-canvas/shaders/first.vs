#version 460 core

layout(location = 0) in vec3 vertex_model_space;
layout(location = 1) in vec2 vertex_uv;

out vec2 uv;

void main() {
  gl_Position.xyz = vertex_model_space;
  gl_Position.w = 1.0;
  uv = vertex_uv;
}
