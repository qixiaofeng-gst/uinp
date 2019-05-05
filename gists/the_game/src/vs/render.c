attribute vec2 vertex;

uniform vec2 top_left;
uniform vec2 bottom_right;
uniform vec2 container_ratio;

varying vec2 coord_ripples;
varying vec2 coord_bg;
void main() {
  coord_bg = mix(top_left, bottom_right, vertex * 0.5 + 0.5);
  coord_bg.y = 1.0 - coord_bg.y;
  coord_ripples = vec2(vertex.x, -vertex.y) * container_ratio * 0.5 + 0.5;
  gl_Position = vec4(vertex.x, -vertex.y, 0.0, 1.0);
}