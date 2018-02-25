#version 330 core

in vec3 outNormal;
in vec4 outColor;

out vec3 color;

varying in vec3 bary;

#extension GL_OES_standard_derivatives : enable
float edgeFactor(){
    vec3 d = fwidth(bary);
    vec3 a3 = smoothstep(vec3(0.0), d * 1.5, bary);
    return min(min(a3.x, a3.y), a3.z);
}

void main()
{
	vec3 tet_color = (dot(outNormal, vec3(0.445435, 0.890871, 0.0890871)) * 0.5 + 0.5) * outColor.rgb;

  	color = mix(vec3(0.0), tet_color, edgeFactor());
}
