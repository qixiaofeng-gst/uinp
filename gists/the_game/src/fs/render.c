precision mediump float;

varying float line_color;

void main() {
  gl_FragColor = vec4(line_color, line_color, line_color, 1.0);
}