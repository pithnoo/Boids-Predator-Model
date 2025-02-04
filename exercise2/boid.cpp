#include "boid.hpp"

Boid::Boid(){
  float x = std::rand() % 100 * 0.01f;
  float y = std::rand() % 100 * 0.01f;
  position = {x, y};

  float ax = std::rand() % 100 * 0.0001f;
  float ay = std::rand() % 100 * 0.0001f;
  acceleration = {ax, ay};

  // initialise position vbo
  glGenBuffers(1, &posVBO);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(boidPos), boidPos, GL_STATIC_DRAW);

  // initialise vao (stores vbo)
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, posVBO);
  glVertexAttribPointer(
			   0,
			   2, GL_FLOAT, GL_FALSE,
			   0,
			   0
			   );
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // delete vbo, as it has now been stored in vao
  glDeleteBuffers(1, &posVBO);
}

void Boid::update(std::vector<Boid>& boids, ShaderProgram* prog, float dt){

  // calculate distances between screen boundaries
  // origin is defined at the center of the screen
  float dRight = 1.f - position.x;
  float dLeft = std::abs(-1.f - position.x);

  // conversion of NDC coordinates to the current screen coordinates
  float dX = std::min(dLeft, dRight) * 720.f;

  if(dX < minDistance){
    if(dLeft < dRight){
      acceleration.x += boundaryForce;
    }
    else{
      acceleration.x -= boundaryForce;
    }
  }

  float dTop = 1.f - position.y;
  float dBottom = std::abs(-1.f - position.y);

  float dY = std::min(dBottom, dTop) * 360.f;

  if(dY < minDistance){
    if(dBottom < dTop){
      acceleration.y += boundaryForce;
    }
    else{
      acceleration.y -= boundaryForce;
    }
  }

  std::vector<Boid> neighbours;

  // for seperation
  std::vector<Boid> closeNeighbours;

  float dx, dy;
  float neighbourDistance;

  // looping over neighbours
  for(auto &b : boids){
	// skip boid that is in exact position
	if(b.position == position)
	  continue;

	// for optimisation, we only need to take 3 neighbour boids here
	if(neighbours.size() >= 3)
	  break;

	// calculating the square of x and y
	// TODO: change accordingly depending on the surface height
	dx = std::pow((position.x - b.position.x) * 640.f, 2);
	dy = std::pow((position.y - b.position.y) * 360.f, 2);

	// pythagorus to find distance between neighbours
	neighbourDistance = std::sqrt(dx + dy);

	if(neighbourDistance <= boidRange){
	  // std::cout << "neighbour: " << neighbourDistance << std::endl;
	  neighbours.push_back(b);

	  /*
	  // if its really close, we gotta avoid it
	  if(neighbourDistance <= avoidDistance){
		closeNeighbours.push_back(b);
	  }
	  */
	}
  }

  Vec2f ruleAcceleration = {0.f, 0.f};

  /*
  // seperation: ensure that boids steer to avoid their flock mates
  Vec2f averageSeperation = {0.f, 0.f};
  for(auto &n : closeNeighbours){
	std::cout << "close neighbour found: " << n.position.x << std::endl;
	averageSeperation += n.position - position;
  }
  averageSeperation /= neighbours.size();

  ruleAcceleration += averageSeperation * 0.0001f;
  */

  // alignment: steer towards the average heading of the flock
  Vec2f averageAcceleration = {0.f, 0.f};
  for(auto &n : neighbours){
	averageAcceleration += n.acceleration;
  }
  averageAcceleration /= neighbours.size();

  ruleAcceleration += (averageAcceleration - position) * 0.0001f;

  // cohesion: steer the boid towards the local center of the flock
  Vec2f averagePosition = {0.f, 0.f};
  for(auto &n : neighbours){
	averagePosition += n.position;
  }
  averagePosition /= neighbours.size();

  ruleAcceleration += (averagePosition - position) * 0.0001f;

  // take average acceleration of the 3
  acceleration += ruleAcceleration / 3;

  // deciding the resulting acceleration and rotation
  position.x += acceleration.x;
  position.y += acceleration.y;

  // the rotation of the boid can be calcualted by the current acceleration vector
  // note: we'll want to lerp towards this
  rotation = std::atan2(acceleration.y, acceleration.x) - (M_PI / 2.0f);

  // we can clear current neighbours once we calculate resultant acceleration
  neighbours.clear();

  // make the matrix for current boid position
  Mat33f boidTransform = make_translation_3H({position.x, position.y}) * make_rotation_3H(rotation);

  glUseProgram(prog->programId());

  // draw the boid in environment
  glBindVertexArray(vao);

  // pass transformation matrix to the vertex shader
  glUniformMatrix3fv(0, 1, GL_TRUE, boidTransform.m);

  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);
}
