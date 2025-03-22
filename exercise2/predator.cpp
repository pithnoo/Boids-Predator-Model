#include "predator.hpp"

Predator::Predator(ShaderProgram *prog){
  predProg = prog;

  // initialise predator VBO and VAOs
  glGenBuffers(1, &posVBO);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glBufferData(GL_ARRAY_BUFFER,3 * sizeof(Vec3f), nullptr,
               GL_DYNAMIC_DRAW);

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
}

void Predator::update(BoidSystem bs, float dt, float predSpeed, float boundaryForce) {

  // find the largest cluster for boid to target
  // can identify the largest cluster, but i gotta set it after
  largestCluster = bs.highestCluster();

  // std::printf("largest cluster: %i\n", largestCluster.clusterCount);

  // update time taken
  timeElapsed += dt;

  //std::printf("current state: %i, time elapsed: %.2f\n", predState, timeElapsed);
  /*
  switch (predState) {
  case IDLE:
	predState = idleState();
  case MARGIN:
	predState = marginState();
  case CENTER:
	predState = centerState();
  }
  */

  idleState();

  // calculate and draw the desired position
  if(largestCluster.clusterCount <= (int)bs.boids.size()){
	velocity += acceleration;
	velocity = normalize(velocity) * predSpeed;
  }
  position += velocity * dt;
  rotation = std::atan2(velocity.y, velocity.x) - (M_PI / 2.0f);

  /*
  position.x += velocity.x * dt;
  position.y += velocity.y * dt;
  */
  //std::printf("position: %.2f, %.2f\n", position.x, position.y); 
  //std::printf("velocity: %.2f, %.2f\n", velocity.x, velocity.y); 


  Mat33f predTransform = make_translation_3H({position.x, position.y}) *
                         make_rotation_3H(rotation);

  std::vector<Vec3f> predBuffer;
  for(auto &v : predPositions){
	Vec3f newPos = predTransform * v;
	predBuffer.emplace_back(newPos);
  }

  // draw call
  draw(predBuffer);
  
  // reset predator acceleration for the next frame
  acceleration = {0.f, 0.f};
}

Predator::state Predator::idleState(){
  if(timeElapsed >= idleTime){
	// decide margin or center
	timeElapsed = 0.f;
	return CENTER;
  }

  // otherwise attempt to match the velocity of the largest flock
  Vec2f clusterVelocity = largestCluster.averageVelocity();

  acceleration += clusterVelocity;

  return IDLE;
}

Predator::state Predator::marginState(){
  if(timeElapsed >= diveTime){
	timeElapsed = 0.f;
	return IDLE;
  }
  // we can target a random edge boid later

  return MARGIN;
}

Predator::state Predator::centerState(){
  if(timeElapsed >= diveTime){
	timeElapsed = 0.f;
	return IDLE;
  }

  Vec2f clusterCenter = largestCluster.averageCenter();
  Vec2f targetVector = normalize(clusterCenter - position);

  // this might set to change to the radius of the cluster (so that it works for varying sizes
  //Vec2f targetPosition = targetVector * leadRule;
  acceleration += targetVector; 

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
