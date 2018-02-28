#version 330 core

in vec3 normal;
in vec2 uv;
in vec3 bary;
uniform int textured;
uniform int rendering_mode;
uniform vec3 wire_color;

uniform sampler2D sampler;

out vec3 color;

#extension GL_OES_standard_derivatives : enable
float edgeFactor(){
    vec3 d = fwidth(bary);
    vec3 a3 = smoothstep(vec3(0.0), d, bary);
    return min(min(a3.x, a3.y), a3.z);
}

void main()
{
	if(textured > 0)
  		color = texture( sampler, uv ).rgb * dot(normal, normalize(vec3(0.5, 1.0, 0.1)));
  	else
  	{
  		if(rendering_mode == 1)
  			color =  mix(vec3(0.0), vec3(0.2) + vec3(0.8) * max(dot(normal, normalize(vec3(0.5, 1.0, 0.1))), 0), edgeFactor());
  		else
			color =  vec3(0.2, 0.2, 0.2) + vec3(0.8,0.8,0.8)  * max(dot(normal, normalize(vec3(0.5, 1.0, 0.1))), 0);
  	}
}