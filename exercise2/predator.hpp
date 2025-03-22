#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <vector>
#include "boid.hpp"

class Predator{
public:
  Vec2f position = { 0.f, 0.f };
  Vec2f velocity = { 0.f, 0.f };
  Vec2f acceleration = { 0.f, 0.f };

  float rotation;

  // speed to dive
  float diveDistance;

  // time between attacks
  float idleTime = 3.f;

  // time to dive towards desired position
  float diveTime = .2f;

  enum state : int {
	IDLE,
	MARGIN,
	CENTER,
  };

  // current predator state
  state predState = IDLE;

  // initialise with knowledge of boid system
  Predator(ShaderProgram *prog);

  void update(BoidSystem bs, float dt, float predSpeed, float boundaryForce);

  void draw(std::vector<Vec3f> predBuffer);

  std::vector<Vec3f> predPositions = {
	Vec3f(0.f, 0.01f, 1.f),
	Vec3f(-0.005f, -0.005f, 1.f),
	Vec3f(0.005f, -0.005f, 1.f),
  };

  float const predColor[3] = { 1.f, 0.f, 0.f };

  // time elapsed for predator
  float timeElapsed = 0.f;

private:
  // easier for me to have a seperate draw call for predators, which hunt roughly at pairs max
  GLuint posVBO;
  GLuint vao;

  BoidCluster largestCluster;

  state idleState();
  state marginState();
  state centerState();

  void changeState(state newState);

  // reference to current shader program
  ShaderProgram *predProg;
  std::vector<BoidCluster> clusters;

  // the ending position of the predator after targetting the center
  float leadRule = 1.3f;
};

#endif    
