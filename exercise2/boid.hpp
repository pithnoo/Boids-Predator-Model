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

  // boid initial colour
  float const boidColor[3] = { 0.f, 0.5f, 1.f };

  // initial boid positions
  float boidPos[6] = {
    0.f, 0.01f,
    -0.005f, -0.005f,
    0.005f, -0.005f,
  };

  // initialise boid
  Boid();

  void update(std::vector<Boid>& boids, ShaderProgram* prog, float dt, float boidSpeed, float seperationFactor, float alignmentFactor, float cohesionFactor, float boundaryForce);

private:
  // minimum distance before rotating against boundary
  float minDistance = 50.f;

  // factor for acceleration applied at boundary
  float boundaryForce = 0.01f;

  // stop boids from colliding
  float seperationFactor = 0.2f;

  // range to recognise boids that are too close (for seperation)
  float avoidDistance = 10.f;

  // controls boids overall alignment
  float alignmentFactor = 0.1f;

  // control the boids urge to turn towards it's local center
  float cohesionFactor = 0.001f;

  // range to recognise other neighbouring boids
  float boidRange = 50.f;

  // highest speed of a boid
  float boidSpeed = 0.01f;

  // the boid's acceleration, which will change depending on bounds
  Vec2f acceleration = { 0.f, 0.f };
  Vec2f velocity = { 0.f, 0.f };

  // what direction the boid is pointing at
  float rotation = 0;

  GLuint vao = 0;
  GLuint posVBO = 0;
};

#endif
