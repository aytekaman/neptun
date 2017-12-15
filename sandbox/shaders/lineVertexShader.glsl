#version 330 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 color;

out vec4 outColor;

uniform mat4 MVP;
  
void main()
{
	outColor = color;

  gl_Position =  MVP * vec4(vertexPosition, 1);
  //gl_Position.z -= 0.01f;

}

