#version 330 core

in vec4 outColor;

out vec4 final_color;

void main()
{
  final_color = outColor;

  //final_color.r = gl_FragCoord.z / 1.0f; 
}