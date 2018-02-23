#version 330 core

in vec3 normal;
in vec2 uv;
varying in vec3 bary;
uniform int textured;
uniform int rendering_mode;

uniform sampler2D sampler;

out vec3 color;

#extension GL_OES_standard_derivatives : enable
float edgeFactor(){
    vec3 d = fwidth(bary);
    vec3 a3 = smoothstep(vec3(0.0), d*1.5, bary);
    return min(min(a3.x, a3.y), a3.z);
}

void main()
{
	if(textured > 0)
  		color = texture( sampler, uv ).rgb * dot(normal, normalize(vec3(0.5, 1.0, 0.1)));
  	else
  	{
  		if(rendering_mode == 1 && any(lessThan(bary, vec3(1))))
  			color =  mix(vec3(0.0, 0.8, 0.8), vec3(1.0,1.0,1.0) * dot(normal, normalize(vec3(0.5, 1.0, 0.1))), edgeFactor());
  		else
			color =  vec3(1.0,1.0,1.0) * dot(normal, normalize(vec3(0.5, 1.0, 0.1)));
  	}
}