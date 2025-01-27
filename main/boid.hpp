#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <glad/glad.h>

#include <vector>
#include <GL/glext.h>
#include <iostream>
#include <math.h>

#include "../vmlib/vec2.hpp"
#include "../vmlib/mat33.hpp"
#include "../support/program.hpp"


class Boid {
public:
  // central position of the boid
  Vec2f position;

  // vector split of the resultant speed
  float speedX;
  float speedY;
  float dt;

  // boid initial colour
  float const boidColor[3] = { 0.f, 0.5f, 1.f };

  // initial boid positions
  float boidPos[6] = {
    0.f, 0.05f,
    -0.025f, -0.05f,
    0.025f, -0.05f,
  };

  // minimum distance before rotating against boundary
  float minDistance;

  // how fast we want boid to move
  float speed;

  // initialise boid
  Boid(float d, float s);

  void update(ShaderProgram* prog, float dt);

  // possible call function to get current position of boid

private:
  // resultant x and y (comparing it with edges of screen)
  float disX;
  float disY;

  // what direction the boid is pointing at
  float rotation = 0;

  GLuint vao = 0;
  GLuint posVBO = 0;
};

#endif
