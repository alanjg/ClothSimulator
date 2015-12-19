#ifndef MATH_H
#define MATH_H
#include "constants.h"

namespace ajg {
namespace physics {

	
	const Scalar PI = 3.1415926535897932384626433832795f;
//	const Scalar E = 2.71828182845904523536f;
	const float FLOAT_TOLERANCE = 1e-4f;
	const double DOUBLE_TOLERANCE = 1e-8;

	bool IsEqual(float lhs,float rhs,float tolerance = FLOAT_TOLERANCE);
	bool IsEqual(double lhs,double rhs,double tolerance = DOUBLE_TOLERANCE);
	bool IsEqual(const Vector3& lhs,const Vector3& rhs);
	bool IsZero(float lhs,float tolerance = FLOAT_TOLERANCE);
	bool IsZero(double lhs,double tolerance = DOUBLE_TOLERANCE);
	

}
}

#endif
