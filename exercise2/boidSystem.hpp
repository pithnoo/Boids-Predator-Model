#include <vector>
#include <iostream>

#include "boid.hpp"
#include "../support/program.hpp"

class BoidSystem {
public:
  BoidSystem();
  void update(ShaderProgram* prog, float dt);
  
private:
  GLuint posVBO;

  float const boidColor[3] = { 0.f, 0.5f, 1.f };

  float boidPos[6] = {
    0.f, 0.01f,
    -0.005f, -0.005f,
    0.005f, -0.005f,
  };

};
