#include "boid.hpp"
#include <GL/glext.h>

Boid::Boid(float d, float s){
  minDistance = d;
  speed = s;

  // initialise position vbo
  glGenBuffers(1, &posVBO);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(boidPos), boidPos, GL_STATIC_DRAW);

  // initialise vao (stores vbo)
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glVertexAttribPointer(
			   0,
			   2, GL_FLOAT, GL_FALSE,
			   0,
			   0
			   );
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // delete vbo, as it has now been stored in vao
  glDeleteBuffers(1, &posVBO);
}

void Boid::update(ShaderProgram* prog, float dt){
  // calculate distances between screen boundaries
  disX = (1280 - position.x) - position.x;
  disY = (720 - position.y) - position.y;

  if(disX <= minDistance || disY <= minDistance){
	// rotate boid appropriately
	rotation = std::tan(disY / disX);

	// direction to move boid
	speedX = cos(rotation) * speed;
	speedY = sin(rotation) * speed;
  }

  position.x += speedX;
  position.y += speedY;

  // TODO: calculate acceleration against other neighbouring boids

  // make the matrix for current boid position
  Mat33f boidTransform = make_translation_3H({position.x, position.y}) * make_rotation_3H(rotation);

  glUseProgram(prog->programId());

  // draw the boid in environment
  glBindVertexArray(vao);

  // pass transformation matrix to the vertex shader
  glUniformMatrix3fv(0, 1, GL_TRUE, boidTransform.m);

  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);

}
