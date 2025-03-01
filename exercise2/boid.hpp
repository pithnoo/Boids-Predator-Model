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

  // indication boid is close to boundary (we want boids to move away from it)
  bool atBoundary = false;

  // for the DBScan algo
  bool inCluster = false;
  bool corePoint = false;

  // identify if the boid is at an edge of a flock (primary target)
  bool isEdge = false;

  // boid initial colour
  float const boidColor[3] = { 0.f, 0.5f, 1.f };

  // initial boid positions
  std::vector<Vec3f> boidPositions = {
	Vec3f(0.f, 0.01f, 1.f),
	Vec3f(-0.005f, -0.005f, 1.f),
	Vec3f(0.005f, -0.005f, 1.f),
  };

  // initialise boid
  Boid();

  Mat33f update(std::vector<Boid>& boids, ShaderProgram* prog, float dt, float boidSpeed, float seperationFactor, float alignmentFactor, float cohesionFactor, float boundaryForce, float steeringFactor);

private:
  // minimum distance before rotating against boundary
  float minDistance = 50.f;

  // range to recognise boids that are too close (for seperation)
  float avoidDistance = 10.f;

  // range to recognise other neighbouring boids
  float boidRange = 50.f;

  // the boid's acceleration, which will change depending on bounds
  Vec2f acceleration = { 0.f, 0.f };
  Vec2f velocity = { 0.f, 0.f };

  // what direction the boid is pointing at
  float rotation = 0;
};

class BoidCluster {
public:
  // boids at the edge for a primary target
  std::vector<Boid> edgeBoids;
  std::vector<Vec2f> clusterCentroid;
  std::vector<Vec2f> clusterVelocity;
  std::vector<int> clusterCount;
};

class BoidSystem {
public:
  // boids tp update
  std::vector<Boid> boids;

  // for the predator to identify
  std::vector<BoidCluster> clusters;

  // enable pausing the boids system currently
  bool isPaused;

  BoidSystem(int N);

  void update(ShaderProgram* prog, float dt, float boidSpeed, float seperationFactor, float alignmentFactor, float cohesionFactor, float boundaryForce, float steeringFactor);
  
private:
  GLuint vao = 0;
  GLuint posVBO = 0;
  GLuint insVBO = 0;

  float const boidColor[3] = { 0.f, 0.5f, 1.f };
};

#endif
