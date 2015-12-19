#include "math.h"

namespace ajg {
namespace physics {

bool IsEqual(float lhs,float rhs,float tolerance)
{
	return fabsf(lhs - rhs) < tolerance;
}

bool IsEqual(double lhs,double rhs,double tolerance)
{
	return fabs(lhs - rhs) < tolerance;
}

bool IsZero(float lhs,float tolerance)
{
	return fabsf(lhs) < tolerance;
}

bool IsZero(double lhs,double tolerance)
{
	return fabs(lhs) < tolerance;
}

bool IsEqual(const Vector3& lhs,const Vector3& rhs)
{
	return IsEqual(lhs[0],rhs[0]) && IsEqual(lhs[1],rhs[1]) && IsEqual(lhs[2],rhs[2]);
}

}
}
