#version 330 core

layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 anormal;
layout(location = 2) in vec2 auv;
layout(location = 3) in vec3 abary;


out vec3 normal;
out vec2 uv;
out vec3 bary;

//out int t;
  
// Values that stay constant for the whole mesh.
uniform mat4 MVP;

uniform mat4 N;
//uniform int textured;
  
void main(){
  // Output position of the vertex, in clip space : MVP * position
  gl_Position =  MVP * vec4(vertexPosition_modelspace, 1);
  normal = vec3(N * vec4(anormal, 1));
  uv = auv;
  bary = abary;
  //t = textured;
}