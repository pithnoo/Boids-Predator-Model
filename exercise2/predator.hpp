#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <glad/glad.h>
#include <GL/glext.h>
#include <vector>

#include "boid.hpp"

class Predator{
public:
  // minimum distance to maintain before attack
  float scoutDistance;

  // time between attacks
  float idleTime;

  enum state : int {
	IDLE,
	MARGIN,
	CENTER,
  };

  // current predator state
  state predState = IDLE;

  // initialise with knowledge of boid system
  Predator(BoidSystem *bs, ShaderProgram *prog);

  Mat33f update(float dt, float predSpeed, float boundaryForce);

  void draw();

  float const predColor[3] = { 1.f, 0.f, 0.f };

private:
  // easier for me to have a seperate draw call for predators, which hunt roughly at pairs max
  GLuint posVBO;
  GLuint vao;

  // reference to current shader program
  ShaderProgram *predProg;
  std::vector<BoidCluster> clusters;

  // time elapsed for predator
  float timeElapsed = 0.f;
};

#endif    
