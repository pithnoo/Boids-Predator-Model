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
  v2fColor = vec3(0, 0, 0);

  // new position
  vec3 tPosition = transform * vec3(iPosition.xy, 1.0);

  // i need a projection matrix here
  gl_Position = vec4( tPosition.xy, 0.0, 1.0 );
}
