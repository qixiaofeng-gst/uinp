precision mediump float;

uniform sampler2D sampler_bg;
uniform sampler2D sampler_ripples;
uniform vec2 delta;
uniform float perturbance;

varying vec2 coord_ripples;
varying vec2 coord_bg;

void main() {
  float height = texture2D(sampler_ripples, coord_ripples).r;
  float heightX = texture2D(sampler_ripples, vec2(coord_ripples.x + delta.x, coord_ripples.y)).r;
  float heightY = texture2D(sampler_ripples, vec2(coord_ripples.x, coord_ripples.y + delta.y)).r;
  vec3 dx = vec3(delta.x, heightX - height, 0.0);
  vec3 dy = vec3(0.0, heightY - height, delta.y);
  vec2 offset = -normalize(cross(dy, dx)).xz;
  float specular = pow(max(0.0, dot(offset, normalize(vec2(-0.6, 1.0)))), 4.0);
  gl_FragColor = texture2D(sampler_bg, coord_bg + offset * perturbance) + specular;
}