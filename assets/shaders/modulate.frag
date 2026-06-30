#version 430

// from vertex shader
// the inputs of the fragment shader must follow the vertex shader
in vec3 v2fColor;

// uniform = same for all fragments
// from location 0, all fragments will have the same base colour

layout( location = 0 ) out vec3 oColor;

void main()
{
  oColor = v2fColor;
}
