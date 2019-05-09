precision mediump float;

varying float line_color;
varying vec2 pos;

void main() {
  float alpha = length(pos);
  if (alpha < 1.0) {
    alpha = 1.0 - alpha;
  } else {
    alpha = 0.0;
  }
  gl_FragColor = vec4(1.0, 1.0, 1.0, alpha);
}