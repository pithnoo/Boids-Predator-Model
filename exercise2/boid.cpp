#include "boid.hpp"

Boid::Boid(){
  // calculate boid size (co-ordinates should be decided so scale of the boid?)
  float x = (std::rand() % 1000 * 0.002f) - 1.0f;
  float y = (std::rand() % 1000 * 0.002f) - 1.0f;
  position = {x, y};

  float ax = std::rand() % 10000 * 0.01f;
  float ay = std::rand() % 10000 * 0.01f;
  velocity = {ax, ay};

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

void Boid::update(std::vector<Boid>& boids, ShaderProgram* prog, float dt, float boidSpeed, float seperationFactor, float alignmentFactor, float cohesionFactor, float boundaryForce, float steeringFactor){

  // calculate distances between screen boundaries
  // origin is defined at the center of the screen
  float dRight = 1.f - position.x;
  float dLeft = std::abs(-1.f - position.x);

  // conversion of NDC coordinates to the current screen coordinates
  float dX = std::min(dLeft, dRight) * 640.f;

  if(dX < minDistance){
	atBoundary = true;
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
	atBoundary = true;
    if(dBottom < dTop){
      acceleration.y += boundaryForce;
    }
    else{
      acceleration.y -= boundaryForce;
    }
  }

  std::vector<Boid> neighbours;

  // for seperation rule
  std::vector<Boid> closeNeighbours;

  float dx, dy;
  float neighbourDistance;

  // looping over neighbours
  for(auto &b : boids){
	// skip boid that is in exact position
	if(b.position == position)
	  continue;

	// calculating the square of x and y
	// TODO: change accordingly depending on the surface height
	dx = std::pow((position.x - b.position.x) * 640.f, 2);
	dy = std::pow((position.y - b.position.y) * 360.f, 2);

	// pythagorus to find distance between neighbours
	neighbourDistance = std::sqrt(dx + dy);
	
	if(neighbourDistance <= boidRange && b.atBoundary == false){
	  neighbours.emplace_back(b);

	  // if its really close, we gotta avoid it
	  if(neighbourDistance <= avoidDistance){
		closeNeighbours.emplace_back(b);
	  }
	}

	// reducing the number of iterations, as we don't need to take the whole flock
	if(closeNeighbours.size() >= 3)
	  break;
  }

  Vec2f ruleAcceleration = {0.f, 0.f};
  
  // seperation: ensure that boids steer to avoid their flock mates
  Vec2f averageSeperation = {0.f, 0.f};

  for(auto &n : closeNeighbours){
	// normalize this vector to prevent further boids from having a greater influence
	averageSeperation += normalize(position - n.position);
  }

  if(closeNeighbours.size() > 0)
	ruleAcceleration += normalize(averageSeperation) * seperationFactor;

  // alignment: steer towards the average heading of the flock
  Vec2f averageAlignment = {0.f, 0.f};

  for(auto &n : neighbours){
	averageAlignment += n.velocity;
  }

  // if we take the average, then higher velocities will take priority in direction
  if(neighbours.size() > 0)
	ruleAcceleration += normalize(averageAlignment) * alignmentFactor;
  
  // cohesion: steer the boid towards the local center of the flock
  Vec2f averagePosition = {0.f, 0.f};

  // take influence of only boids that are not at the boundary
  for(auto &n : neighbours){
	averagePosition += n.position;
  }

  // accounts for if no neighbours (i.e. prevents division by 0)
  if(neighbours.size() > 0)
	averagePosition /= neighbours.size();

  // why check if this isn't 0?
  if(neighbours.size() > 0)
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

  // note: add delta time to this
  position.x += velocity.x * dt;
  position.y += velocity.y * dt;

  // the rotation of the boid can be calcualted by the current acceleration vector
  rotation = std::atan2(velocity.y, velocity.x) - (M_PI / 2.0f);

  // we can clear current neighbours once we calculate resultant acceleration
  neighbours.clear();
  closeNeighbours.clear();

  // reset acceleration
  acceleration = {0.f, 0.f};

  // set false until proven otherwise on the next frame
  atBoundary = false;

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

void BoidSystem::update(ShaderProgram* prog, float dt, float boidSpeed, float seperationFactor, float alignmentFactor, float cohesionFactor, float boundaryForce, float steeringFactor){

  glUseProgram(prog);

  std::vector<Vec3f> boidTriangles;

  if(!isPaused){
	// update boids
	for(auto &b : boids){
          b.update(
				   boids,
				   prog,
				   dt,
				   boidSpeed,
				   seperationFactor,
				   alignmentFactor,
				   cohesionFactor,
				   boundaryForce,
				   steeringFactor
				   );

		  // emplace back the new triangle position
	}
  }

  // make posVBO

  // bind vao to store
  //glBindVertexArray(vao);

  // draw arrays
  // glDrawArrays( GL_TRIANGLES, 0, triangles.size() );

  glBindVertexArray( 0 );
  glBindBuffer( GL_ARRAY_BUFFER, 0);
}
