#version 430

// I'm guessing here that he defined i for 'input'
// here the location is specified corresponding to the VAO
layout( location = 0 ) in vec2 iPosition;
layout( location = 1 ) in vec3 iColor;

layout( location = 0 ) uniform mat3 transform;

// output for fragment shader
out vec3 v2fColor;

void main()
{
  v2fColor = iColor;
  gl_Position = transform * vec4( iPosition.xy, 0.0, 1.0 );
}
