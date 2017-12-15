#version 330 core

in vec3 outNormal;
in vec4 outColor;

out vec3 color;

void main()
{
  color = (dot(outNormal, vec3(0.445435, 0.890871, 0.0890871)) * 0.5 + 0.5) * outColor.rgb;
}