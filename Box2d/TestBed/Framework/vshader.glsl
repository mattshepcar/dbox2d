// Vertex Shader – file "minimal.vert"

#version 130

in  vec3 in_Position;
in  vec3 in_Color;
out vec3 ex_Color;

void main(void)
{
	ex_Color = in_Color;
	gl_Position = vec4(in_Position * .1 - vec3(.5,.5,0), 1.0);
}
