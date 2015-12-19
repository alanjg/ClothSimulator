#include "Particle.h"
#include "IForce.h"

namespace ajg {
namespace physics {

void Particle::AccumulateForce()
{
	force = Vector3::Zero();
	for(unsigned int i = 0; i < forces.size(); i++)
	{
		force += forces[i]->Apply();
	}
}

void Particle::AddForce(IForce* f)
{
	forces.push_back(f);
}

}
}
