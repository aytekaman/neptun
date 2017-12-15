#version 330 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 vertexColor;

out vec3 outNormal;
out vec4 outColor;

uniform mat4 MVP;
  
void main()
{
	outNormal = vertexNormal;
	outColor = vertexColor;

  	gl_Position =  MVP * vec4(vertexPosition, 1);
}

