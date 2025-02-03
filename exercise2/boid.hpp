#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <glad/glad.h>

#include <vector>
#include <GL/glext.h>
#include <iostream>
#include <math.h>
#include <cmath>
#include <cstdlib>

#include "../vmlib/vec2.hpp"
#include "../vmlib/mat33.hpp"
#include "../support/program.hpp"

class Boid {
public:
  // central position of the boid
  Vec2f position = {0.f, 0.f};

  // vector split of the resultant speed
  float dt;

  // boid initial colour
  float const boidColor[3] = { 0.f, 0.5f, 1.f };

  // initial boid positions
  float boidPos[6] = {
    0.f, 0.03f,
    -0.01f, -0.03f,
    0.01f, -0.03f,
  };

  float boundaryForce = 0.001f;

  // how fast we want boid to move
  float speed;

  // initialise boid
  Boid();

  void update(const std::vector<Boid>& boids, ShaderProgram* prog, float dt);

  // possible call function to get current position of boid

private:
  // minimum distance before rotating against boundary
  float minDistance = 40.f;

  // range to recognise other neighbouring boids
  float boidRange = 5.f;

  // the boid's acceleration, which will change depending on bounds
  Vec2f acceleration = { 0.01f, 0.01f };

  // resultant x and y (comparing it with edges of screen)
  float disX;
  float disY;

  // what direction the boid is pointing at
  float rotation = 0;

  GLuint vao = 0;
  GLuint posVBO = 0;
};

#endif
