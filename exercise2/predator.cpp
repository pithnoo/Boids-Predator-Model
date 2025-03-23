#include "predator.hpp"

Predator::Predator(ShaderProgram *prog) {
  predProg = prog;

  position = {0.f, 0.f};

  // initialise predator VBO and VAOs
  glGenBuffers(1, &posVBO);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(Vec3f), nullptr, GL_DYNAMIC_DRAW);

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
}

void Predator::update(BoidSystem bs, float dt, float predSpeed, float diveSpeed,
                      float boundaryForce, bool isPaused) {
  // update time taken (for states)
  timeElapsed += dt;

  switchElapsed += dt;
  if(switchElapsed >= switchTime){
	largestCluster = bs.highestCluster();
	switchElapsed = 0.f;
  }

  float dRight = 1.f - position.x;
  float dLeft = std::abs(-1.f - position.x);

  float dX = std::min(dLeft, dRight) * 640.f;
  if (dX < minDistance) {
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
    if (dBottom < dTop) {
      acceleration.y += boundaryForce;
    } else {
      acceleration.y -= boundaryForce;
    }
  }

  switch (predState) {
  case IDLE:
	// reset dive
    hasDived = false;
    predState = idleState();
    break;
  case MARGIN:
    predState = marginState();
    break;
  case CENTER:
    predState = centerState(predSpeed);
    break;
  }

  // calculate and draw the desired position
  // only when clusters have had time to form
  if (largestCluster.clusterCount <= (int)bs.boids.size() && largestCluster.clusterCount >= 0) {
	// temporary steering factor
    velocity += acceleration * 0.05f;

	if(!hasDived){
	  diveAccel -= 0.01f;
	  predSpeed = std::max(diveAccel, predSpeed);
	}
	else{
	  diveAccel += 0.02f;
	  predSpeed = std::min(diveAccel, maxSpeed);
	}

	velocity = normalize(velocity) * predSpeed;
  }

  if (!isPaused) {
    position += velocity * dt;
    rotation = std::atan2(velocity.y, velocity.x) - (M_PI / 2.0f);
  }

  Mat33f predTransform = make_translation_3H({position.x, position.y}) *
                         make_rotation_3H(rotation);

  std::vector<Vec3f> predBuffer;
  for (auto &v : predPositions) {
    Vec3f newPos = predTransform * v;
    predBuffer.emplace_back(newPos);
  }

  // draw call
  draw(predBuffer);

  // reset predator acceleration for the next frame
  acceleration = {0.f, 0.f};
}

Predator::state Predator::idleState() {
  Vec2f clusterCenter = largestCluster.averageCenter();
  float dx = (position.x - clusterCenter.x) * 640.f;
  float dy = (position.y - clusterCenter.y) * 360.f;
  float predatorDistance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

  if(predatorDistance >= largestCluster.clusterRadius() + 80.f){
    Vec2f targetVector = normalize(clusterCenter - position);
    acceleration += targetVector;
  }
  else{
	// otherwise attempt to match the velocity of the largest flock
	Vec2f clusterVelocity = largestCluster.averageVelocity();
	acceleration += clusterVelocity;
	// only dive when in the right distance
	if (timeElapsed >= idleTime) {
	  // decide margin or center?
	  timeElapsed = 0.f;
	  return CENTER;
	}
  }

  return IDLE;
}

Predator::state Predator::marginState() {
  if (timeElapsed >= diveTime) {
    timeElapsed = 0.f;
    return IDLE;
  }
  // we can target a random edge boid later

  return MARGIN;
}

Predator::state Predator::centerState(float predSpeed) {
  if (timeElapsed >= diveTime) {
    timeElapsed = 0.f;
    return IDLE;
  }

  // this might set to change to the radius of the cluster (so that it works for
  // varying sizes
  if (!hasDived) {
    Vec2f clusterCenter = largestCluster.averageCenter();
    Vec2f targetVector = normalize(clusterCenter - position);
    acceleration += targetVector;
	diveAccel = predSpeed;
    hasDived = true;
  }

  return CENTER;
}

void Predator::draw(std::vector<Vec3f> predBuffer) {
  glUseProgram(predProg->programId());

  glUniform3f(1, predColor[0], predColor[1], predColor[2]);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glBufferSubData(GL_ARRAY_BUFFER, 0, predBuffer.size() * sizeof(Vec3f),
                  predBuffer.data());
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // bind vao to store
  glBindVertexArray(vao);

  // draw arrays
  glDrawArrays(GL_TRIANGLES, 0, predBuffer.size());
  glBindVertexArray(0);
}
