#version 330 core

in vec3 normal;
in vec2 uv;
uniform int textured;

uniform sampler2D sampler;


out vec3 color;
void main(){
if(textured > 0)
  color = texture( sampler, uv ).rgb * dot(normal, normalize(vec3(0.5, 1.0, 0.1)));
  else
  color =  vec3(1.0,1.0,1.0) * dot(normal, normalize(vec3(0.5, 1.0, 0.1)));
}