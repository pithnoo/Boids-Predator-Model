#include "boid.hpp"

Boid::Boid() {
  // calculate boid size (co-ordinates should be decided so scale of the boid?)
  float x = (std::rand() % 1000 * 0.002f) - 1.0f;
  float y = (std::rand() % 1000 * 0.002f) - 1.0f;

  initialPosition = {x, y};
  position = initialPosition;

  float ax = std::rand() % 10000 * 0.01f;
  float ay = std::rand() % 10000 * 0.01f;
  velocity = {ax, ay};
}

Mat33f Boid::update(std::vector<Boid> &boids, Vec2f predatorPosition,
                    ShaderProgram *prog, float dt, float boidSpeed,
                    float boidVision, float predatorFactor,
                    float seperationFactor, float alignmentFactor,
                    float cohesionFactor, float boundaryForce,
                    float steeringFactor, bool predatorActive, bool isPaused) {

  // calculate distances between screen boundaries
  // origin is defined at the center of the screen
  float dRight = 1.f - position.x;
  float dLeft = std::abs(-1.f - position.x);

  // conversion of NDC coordinates to the current screen coordinates
  float dX = std::min(dLeft, dRight) * 640.f;

  if (dX < minDistance) {
    atBoundary = true;
    if (dLeft < dRight) {
      acceleration.x += boundaryForce;
    } else {
      acceleration.x -= boundaryForce;
    }
  }

  float dTop = 1.f - position.y;
  float dBottom = std::abs(-1.f - position.y);

  float dY = std::min(dBottom, dTop) * 360.f;

  if (dY < minDistance) {
    atBoundary = true;
    if (dBottom < dTop) {
      acceleration.y += boundaryForce;
    } else {
      acceleration.y -= boundaryForce;
    }
  }

  float dx, dy;
  float neighbourDistance;

  // reset IDs to find new neighbours
  boidIDs.clear();

  // looping over neighbours
  for (auto &b : boids) {
    // skip boid that is in exact position
    if (b.position == position)
      continue;

    // calculating the square of x and y
    // TODO: change accordingly depending on the surface height
    dx = (position.x - b.position.x) * 640.f;
    dy = (position.y - b.position.y) * 360.f;

    neighbourDistance = euclidean(position, b.position);

    // ignores vision angle
    if (neighbourDistance <= dbDistance) {
      boidIDs.emplace_back(b.id);
    }

    // finding boid vision angle
    Vec2f dv = Vec2f{dx, dy};
    float da = dot(dv, velocity) / (neighbourDistance * length(velocity));

    // TODO: put an angle here for boid sight range
    if (neighbourDistance <= boidRange && acos(da) < boidVision) {
      neighbours.emplace_back(b);

      // if its really close, we gotta avoid it
      if (neighbourDistance <= avoidDistance) {
        closeNeighbours.emplace_back(b);
      }
    }

    // reducing the number of iterations, as we don't need to take the whole
    // flock
    if (closeNeighbours.size() >= 10) {
      break;
    }
  }

  Vec2f ruleAcceleration = {0.f, 0.f};

  // predator: ensure that boids move away from predator above all else
  float predatorDistance = euclidean(position, predatorPosition);

  if (predatorDistance <= 40.f && predatorActive)
    acceleration += normalize(position - predatorPosition) * predatorFactor;

  // seperation: ensure that boids steer to avoid their flock mates
  Vec2f averageSeperation = {0.f, 0.f};

  for (auto &n : closeNeighbours) {
    // normalize this vector to prevent further boids from having a greater
    // influence
    averageSeperation += normalize(position - n.position);
  }

  if (closeNeighbours.size() > 0)
    ruleAcceleration += normalize(averageSeperation) * seperationFactor;

  // alignment: steer towards the average heading of the flock
  Vec2f averageAlignment = {0.f, 0.f};

  for (auto &n : neighbours) {
    averageAlignment += n.velocity;
  }

  // if we take the average, then higher velocities will take priority in
  // direction
  if (neighbours.size() > 0)
    ruleAcceleration += normalize(averageAlignment) * alignmentFactor;

  // cohesion: steer the boid towards the local center of the flock
  Vec2f averagePosition = {0.f, 0.f};

  // take influence of only boids that are not at the boundary
  for (auto &n : neighbours) {
    averagePosition += n.position;
  }

  // accounts for if no neighbours (i.e. prevents division by 0)
  if (neighbours.size() > 0)
    averagePosition /= neighbours.size();

  // why check if this isn't 0?
  if (neighbours.size() > 0)
    ruleAcceleration += normalize(averagePosition - position) * cohesionFactor;

  // average out the influence of the 3 rules
  ruleAcceleration /= 3;

  // determining how much the 3 main rules affect overall acceleration
  acceleration += ruleAcceleration * steeringFactor;

  // deciding the resulting acceleration and rotation
  velocity.x += acceleration.x;
  velocity.y += acceleration.y;

  // normalize velocity to ensure that it does not exceed the boid speed limit
  velocity = normalize(velocity) * boidSpeed;

  // only change when not paused
  if (!isPaused) {
    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    // the rotation of the boid can be calcualted by the current acceleration
    rotation = std::atan2(velocity.y, velocity.x) - (M_PI / 2.0f);
  }

  // we can clear current neighbours once we calculate resultant acceleration
  neighbours.clear();
  closeNeighbours.clear();

  // reset acceleration
  acceleration = {0.f, 0.f};

  // set false until proven otherwise on the next frame
  atBoundary = false;

  // make the matrix for current boid position
  Mat33f boidTransform = make_translation_3H({position.x, position.y}) *
                         make_rotation_3H(rotation);

  return boidTransform;
}

Vec2f BoidCluster::averageCenter() {
  Vec2f clusterCenter = {0.f, 0.f};

  for (auto &b : clusterBoids) {
    clusterCenter += b.position;
  }

  clusterCenter /= clusterBoids.size();

  return clusterCenter;
}

float BoidCluster::clusterRadius() {
  Vec2f cluster = averageCenter();

  if (edgeBoids.size() <= 0)
    return 0.f;

  Boid edgeBoid = edgeBoids[0];
  float radius = euclidean(cluster, edgeBoid.position);

  return radius;
}

Vec2f BoidCluster::averageVelocity() {
  Vec2f clusterVelocity = {0.f, 0.f};

  for (auto &b : clusterBoids) {
    clusterVelocity += b.velocity;
  }

  clusterVelocity /= clusterBoids.size();

  return clusterVelocity;
}

BoidSystem::BoidSystem(int N) : boids(N) {
  int boidID = 0;

  for (auto &b : boids) {
    // assign initial position of boid for reset
    initialPositions.emplace_back(b.position);

    // assign an ID to identify each boid
    b.id = boidID;
    boidID++;
  }

  // there will only be one instance of this, as all boids will be using the
  // same shape
  glGenBuffers(1, &posVBO);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glBufferData(GL_ARRAY_BUFFER, N * 3 * sizeof(Vec3f), nullptr,
               GL_DYNAMIC_DRAW);

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
}

void BoidSystem::resetPositions() {
  for (auto &b : boids) {
    b.position = b.initialPosition;
  }
}

void BoidSystem::update(ShaderProgram *prog, Vec2f predatorPosition, float dt,
                        float boidSpeed, float boidVision, float predatorFactor,
                        float seperationFactor, float alignmentFactor,
                        float cohesionFactor, float boundaryForce,
                        float steeringFactor, bool predatorActive, bool isPaused) {

  std::vector<Vec3f> boidBuffer;

  if (!isPaused)
    clusters.clear();

  for (auto &b : boids) {
    // calculating transformation
    Mat33f transformation =
      b.update(boids, predatorPosition, prog, dt, boidSpeed, boidVision,
	       predatorFactor, seperationFactor, alignmentFactor,
	       cohesionFactor, boundaryForce, steeringFactor, predatorActive, isPaused);

    // 3 vertex points to be added to VBO
    for (auto &v : b.boidPositions) {
      Vec3f newPos = transformation * v;
      boidBuffer.emplace_back(newPos);
    }
    // calculating transformation

    if (!isPaused) {
      // scanning for clusters
      // has the boid been labelled / visited?
      if (b.isVisited)
        continue;

      // is b a core point?
      if (b.boidIDs.size() < 3) {
        // noise point, not border or core point
        continue;
      }

      b.isVisited = true;

      // make a new cluster
      BoidCluster cluster;
      cluster.clusterCount = 0;
      cluster.clusterBoids.clear();

      // add first core point to the cluster
      if (!b.inCluster) {
        b.inCluster = true;
        cluster.clusterBoids.emplace_back(b);
        cluster.clusterCount++;
      }

      size_t i = 0;
      while (i < b.boidIDs.size()) {

        int id = b.boidIDs[i];

        i++;

        if (boids[id].isVisited) {
          continue;
        }

        boids[id].isVisited = true;

        // core point
        if (boids[id].boidIDs.size() >= 2) {
          // add neighbour points to the current search array
          b.boidIDs.insert(b.boidIDs.end(), boids[id].boidIDs.begin(),
                           boids[id].boidIDs.end());
        } else {
          // otherwise, this is a border point
          // this should be known to search edge boids and boids in the cluster
          cluster.edgeBoids.emplace_back(boids[id]);
        }

        if (!boids[id].inCluster) {
          boids[id].inCluster = true;
          cluster.clusterBoids.emplace_back(boids[id]);
          cluster.clusterCount++;
        }
      }

      // add the cluster
      if (cluster.clusterCount > 0) {
        clusters.emplace_back(cluster);
        /*
        std::printf("----------------------------------------------\n");
        std::printf("clusters: %li, cluster size: %i\n", clusters.size(),
        cluster.clusterCount);
        std::printf("----------------------------------------------\n");
        */
      }
    }
  }
  // scanning for clusters

  if (!isPaused) {
    // reset values for next scan
    for (size_t i = 0; i < boids.size(); i++) {
      // reset until proven otherwise
      boids[i].isVisited = false;
      boids[i].isNoise = false;
      boids[i].inCluster = false;
      boids[i].isCore = false;
      // boid ids will be cleared on the next update
    }
  }

  draw(prog, boidBuffer);
}

void BoidSystem::draw(ShaderProgram *prog, std::vector<Vec3f> boidBuffer) {
  glUseProgram(prog->programId());

  glUniform3f(1, boidColor[0], boidColor[1], boidColor[2]);

  // posVBO
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glBufferSubData(GL_ARRAY_BUFFER, 0, boidBuffer.size() * sizeof(Vec3f),
                  boidBuffer.data());
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // bind vao to store
  glBindVertexArray(vao);

  // draw arrays
  glDrawArrays(GL_TRIANGLES, 0, boidBuffer.size());
  glBindVertexArray(0);
  boidBuffer.clear();
}

BoidCluster BoidSystem::highestCluster() {
  BoidCluster maxCluster;
  int currentMax = 0;

  // note that this may be cleared
  for (auto &c : clusters) {
    if (c.clusterCount > currentMax) {
      currentMax = c.clusterCount;
      maxCluster = c;
    }
  }

  return maxCluster;
}
