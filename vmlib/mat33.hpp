#ifndef MAT33
#define MAT33

/*
  NOTE:
  This is for homogeneous coordinates
  3x3 operations are defined here for transformations
  in a 2D space
*/

#include <cmath>
#include "vec3.hpp"
#include "vec2.hpp"

struct Mat33f {
  float m[9];

  float& operator() (int aI, int aJ){
	assert(aI < 3 && aJ < 3);
	return m[aI*3 + aJ];
  }

  float const& operator() (int aI, int aJ) const {
	assert(aI < 3 && aJ < 3);
	return m[aI*3 + aJ];
  }

};

constexpr Mat33f kIdentity33f = { {
	1.f, 0.f, 0.f,
	0.f, 1.f, 0.f,
	0.f, 0.f, 1.f
} };
  
// matrix with matrix
inline Mat33f operator*(Mat33f const& aLeft, Mat33f const& aRight){
  // r is resultant matrix
  Mat33f r{0.f};

  for(int i = 0; i < 3; i++){
	for(int j = 0; j < 3; j++){
	  // initialise index of resultant matrix r
	  r(i, j) = 0.f;
	  for(int k = 0; k < 3; k++){
		r(i, j) += aLeft(i, k) * aRight(k, j);
	  }
	}
  }

  return r;
}

// matrix with vector
inline Vec3f operator*(Mat33f &aLeft, Vec3f &aRight){
  Vec3f r {0.f};

  for (int i = 0; i < 3; ++i) {
	for (int j = 0; j < 3; ++j) {
	  r[i] += aLeft(i, j) * aRight[j];
	}
  }

  return r;	
}

// homogenous rotation
inline Mat33f make_rotation_3H(float aAngle){
  Mat33f r = kIdentity33f;
  float cosA = std::cos(aAngle);
  float sinA = std::sin(aAngle);

  r(0, 0) = cosA;	
  r(0, 1) = -sinA;
  r(1, 0) = sinA;
  r(1, 1) = cosA;

  return r;
}

// homogenous translation (input given as a Vec2f)
inline Mat33f make_translation_3H(Vec2f aTranslation){
  Mat33f r = kIdentity33f; 

  r(0, 2) = aTranslation.x;
  r(1, 2) = aTranslation.y;

  return r;
}

#endif
