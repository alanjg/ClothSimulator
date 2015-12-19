#include "Forces.h"
#include "Particle.h"

namespace ajg {
namespace physics {

Vector3 GravityForce::Apply()
{
	return Vector3(0,-9.8f,0);
}

DragForce::DragForce(Particle& p,Scalar drag):
	particle(p),dragCoefficient(drag)
{
}

Vector3 DragForce::Apply()
{
	return -(particle.position - particle.oldPosition) * dragCoefficient;
}

SpringForce::SpringForce(Particle& p1,Particle& p2,Scalar k):
	particle(p1),other(p2),stiffness(k)
{
}

Vector3 SpringForce::Apply()
{
	return -(particle.position - other.position) * stiffness;
}

WindForce::WindForce(const Vector3& dir):
	direction(dir),count(0)
{
}

Vector3 WindForce::Apply()
{
	count += 0.01f;
	return direction * (sinf(count) + 1);
}

}
}
