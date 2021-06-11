#version 460 core

layout(location = 0) in vec3 vertex_model_space;

smooth out vec3 smooth_color;

void main() {
  gl_Position.xyz = vertex_model_space;
  gl_Position.w = 1.0;
  // smooth_color = abs(vertex_model_space);
  smooth_color = (vertex_model_space + 1.0) * 0.5;
}
