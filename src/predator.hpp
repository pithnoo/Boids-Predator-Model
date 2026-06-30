#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <vector>
#include "boid.hpp"

class Predator{
public:
  // initial position of the predator
  Vec2f initialPosition = { 0.f, 0.f };
  Vec2f position = { 0.f, 0.f };
  Vec2f velocity = { 0.f, 0.f };
  Vec2f acceleration = { 0.f, 0.f };

  float rotation;

  // speed to dive
  float diveDistance;

  // time between attacks
  float idleTime = 3.f;

  // time to switch between flocks
  float switchTime = 1.f;
  float switchElapsed = switchTime;

  float diveAccel = 0.1f;
  float maxSpeed = 0.8f;

  // time to dive towards desired position
  float diveTime = 0.5f;
  bool hasDived = false;

  enum state : int {
	IDLE,
	MARGIN,
	CENTER,
  };

  // current predator state
  state predState = IDLE;

  // initialise with knowledge of boid system
  Predator(ShaderProgram *prog);

  void resetPosition();

  void update(BoidSystem bs, float dt, float predSpeed, float diveSpeed, float boundaryForce, bool isPaused);

  void draw(std::vector<Vec3f> predBuffer);

  std::vector<Vec3f> predPositions = {
	Vec3f(0.f, 0.015f, 1.f),
	Vec3f(-0.006f, -0.006f, 1.f),
	Vec3f(0.006f, -0.006f, 1.f),
  };

  float const predColor[3] = { 1.f, 1.f, 1.f };

  // time elapsed for predator
  float timeElapsed = 0.f;

private:
  // easier for me to have a seperate draw call for predators, which hunt roughly at pairs max
  GLuint posVBO;
  GLuint vao;

  // minimum distance before rotating against boundary
  float minDistance = 50.f;
  float boundaryForce = 0.5f;

  BoidCluster largestCluster;

  state idleState();
  state marginState();
  state centerState(float predSpeed);

  void changeState(state newState);

  // reference to current shader program
  ShaderProgram *predProg;
  std::vector<BoidCluster> clusters;

  // the ending position of the predator after targetting the center
  float leadRule = 1.3f;
};

#endif    
