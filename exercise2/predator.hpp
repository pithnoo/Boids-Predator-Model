#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <glad/glad.h>
#include <GL/glext.h>
#include <vector>

class Predator{
public:
  // single targets outliers, multiple targets clusters
  enum mode : int {
	SINGLE,
	MULTIPLE
  };

  mode huntState = MULTIPLE;

  enum state : int {
	IDLE,
	DIVE,
  };

  state predState = IDLE;

private:
  // easier for me to have a seperate draw call for predators, which hunt roughly at pairs max
  GLuint posVBO;

  float const boidColor[3] = { 1.f, 0.f, 0.f };
};

#endif    
