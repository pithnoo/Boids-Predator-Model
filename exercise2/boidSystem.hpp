#ifndef BOID_SYSTEM_HPP
#define BOID_SYSTEM_HPP

#include <vector>
#include <GL/glext.h>
#include <iostream>

#include "boid.hpp"
#include "../support/program.hpp"

class BoidSystem {
public:
  // boids tp update
  std::vector<Boid> boids;

  // enable pausing the boids system currently
  bool isPaused;

  BoidSystem();

  void update(ShaderProgram* prog, float dt);
  
private:
  GLuint vao = 0;
  GLuint posVBO = 0;

  float const boidColor[3] = { 0.f, 0.5f, 1.f };

  float boidPos[6] = {
    0.f, 0.01f,
    -0.005f, -0.005f,
    0.005f, -0.005f,
  };

};

#endif
