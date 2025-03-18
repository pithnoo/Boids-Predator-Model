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

  state predState = IDLE;

private:
  // easier for me to have a seperate draw call for predators, which hunt roughly at pairs max
  GLuint posVBO;
  GLuint vao;

  float const predColor[3] = { 1.f, 0.f, 0.f };
};

#endif    
