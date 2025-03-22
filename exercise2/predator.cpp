#include "predator.hpp"

Predator::predator(){

}

Mat33f Predator::update(float dt, float predSpeed, float boundaryForce) {
  switch (predState) {
  case IDLE:
    break;
  case MARGIN:
    break;
  case CENTER:
    break;
  }
}

Mat33f idleState(float dt, float predSpeed){
}

Mat33f marginState(float dt, float predSpeed){
}

Mat33f centerState(float dt, float predSpeed){
}

void draw(){}

void changeState(){
  // reset time elapsed
  timeElapsed = 0.f;
}
