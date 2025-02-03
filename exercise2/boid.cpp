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
  // origin is defined at the center of the screen

  float dRight = 1.f - position.x;
  float dLeft = std::abs(-1.f - position.x);

  float dX = std::min(dLeft, dRight) * 720.f;

  if(dX < minDistance){
    if(dLeft < dRight){
      acceleration.x += boundaryForce;
    }
    else{
      acceleration.x -= boundaryForce;
    }
  }

  float dTop = 1.f - position.y;
  float dBottom = std::abs(-1.f - position.y);

  float dY = std::min(dBottom, dTop) * 360.f;

  if(dY < minDistance){
    // std::cout << "acting on distance:" << dY << "\r";
    if(dBottom < dTop){
      acceleration.y += boundaryForce;
    }
    else{
      acceleration.y -= boundaryForce;
    }
  }

  // deciding the resulting acceleration and rotation
  position.x += acceleration.x;
  position.y += acceleration.y;

  // rotation = std::atan(acceleration.y / acceleration.x);

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
