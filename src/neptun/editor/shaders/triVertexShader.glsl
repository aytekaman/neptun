#version 330 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 vertexColor;
layout(location = 3) in vec3 aBary;

out vec3 outNormal;
out vec4 outColor;
out vec3 bary;

uniform mat4 MVP;
  
void main()
{
	outNormal = vertexNormal;
	outColor = vertexColor;
	bary = aBary;

  	gl_Position =  MVP * vec4(vertexPosition, 1);
}

