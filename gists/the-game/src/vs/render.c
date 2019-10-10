attribute vec3 vertex;

uniform vec2 canvas_size;

varying float line_color;

void main() {
  vec2 one = vec2(1.0, -1.0);
  vec2 tmp = vec2(vertex.x / canvas_size.x, -vertex.y / canvas_size.y) * 2.0 - one;
  if (0.0 == vertex.z) {
    line_color = 0.5;
  } else {
    line_color = 1.0;
  }
  
  gl_Position = vec4(tmp.x, tmp.y, 0.0, 1.0);
}