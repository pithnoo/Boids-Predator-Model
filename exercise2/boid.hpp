#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <glad/glad.h>
#include <GL/glext.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <vector>

#include "../support/program.hpp"
#include "../vmlib/mat33.hpp"
#include "../vmlib/vec2.hpp"

class Boid {
public:
  // allowing identification of each boid, this will be assigned from BoidSystem
  int id = 0;

  // central position of the boid
  Vec2f position = {0.f, 0.f};

  // current velocity of boid
  Vec2f velocity = {0.f, 0.f};

  // what direction the boid is pointing at
  float rotation = 0;

  // indication boid is close to boundary (we want boids to move away from it)
  bool atBoundary = false;

  // for the DBScan algo
  std::vector<int> boidIDs;

  // set until proven otherwise
  bool isNoise = false;
  bool isVisited = false;
  bool isCore = false;
  bool inCluster = false;

  // boid initial colour
  float const boidColor[3] = {0.f, 0.5f, 1.f};

  // initial boid positions
  std::vector<Vec3f> boidPositions = {
      Vec3f(0.f, 0.01f, 1.f),
      Vec3f(-0.005f, -0.005f, 1.f),
      Vec3f(0.005f, -0.005f, 1.f),
  };

  // initialise boid
  Boid();

  Boid &operator=(const Boid &boid) {
    // boid properties
    position = boid.position;
    rotation = boid.rotation;
    atBoundary = boid.atBoundary;
    acceleration = boid.acceleration;
    velocity = boid.velocity;
    // DBscan properties
    boidIDs = boid.boidIDs;
    isNoise = boid.isNoise;
    isVisited = boid.isVisited;
    isCore = boid.isCore;
    inCluster = boid.inCluster;

    return *this;
  }

  Mat33f update(std::vector<Boid> &boids, Vec2f predatorPosition,
                ShaderProgram *prog, float dt, float boidSpeed,
                float predatorFactor, float seperationFactor,
                float alignmentFactor, float cohesionFactor,
                float boundaryForce, float steeringFactor, bool isPaused);

private:
  std::vector<Boid> neighbours;
  std::vector<Boid> closeNeighbours;

  float visionAngle = (5 * M_PI) / 6;

  // float visionAngle = 0.f;

  // minimum distance before rotating against boundary
  float minDistance = 50.f;

  // range to recognise boids that are too close (for seperation)
  float avoidDistance = 10.f;

  // distance to recognise neighbours (for DBscan)
  float dbDistance = 14.f;

  // range to recognise other neighbouring boids (Note: also for DBscan)
  float boidRange = 50.f;

  // the boid's acceleration, which will change depending on bounds
  Vec2f acceleration = {0.f, 0.f};
};

class BoidCluster {
public:
  // boids at the edge for a primary target
  std::vector<Boid> edgeBoids;
  std::vector<Boid> clusterBoids;

  // return average centroid of boids
  Vec2f averageCenter();

  // return radius of the cluster (center to edge)
  float clusterRadius();

  // return average velocity of boids
  Vec2f averageVelocity();

  Vec2f clusterCentroid;
  Vec2f clusterVelocity;

  // this is decided during db scan
  int clusterCount;
};

class BoidSystem {
public:
  // boids tp update
  std::vector<Boid> boids;

  // for the predator to identify
  std::vector<BoidCluster> clusters;

  BoidSystem(int N);

  void update(ShaderProgram *prog, Vec2f predatorPosition, float dt,
              float boidSpeed, float predatorFactor, float seperationFactor,
              float alignmentFactor, float cohesionFactor, float boundaryForce,
              float steeringFactor, bool isPaused);

  BoidCluster highestCluster();

  void draw(ShaderProgram *prog, std::vector<Vec3f> boidBuffer);

private:
  GLuint vao = 0;
  GLuint posVBO = 0;

  float const boidColor[3] = {0.f, 0.5f, 1.f};
};

#endif
