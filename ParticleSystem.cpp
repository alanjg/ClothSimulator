#include "ParticleSystem.h"
#include <iostream>
#include <algorithm>
#include <functional>
#include "Particle.h"
#include "IConstraint.h"

namespace ajg {
namespace physics {

void ParticleSystem::Update(Scalar time)
{
	AccumulateForces();
	Verlet(time);
	SatisfyConstraints();
}

void ParticleSystem::AddParticle(const Vector3& position,const Vector3& velocity)
{
	particles.push_back(new Particle(position,velocity));
}

void ParticleSystem::AddParticle(Particle* particle)
{
	particles.push_back(particle);
}

void ParticleSystem::AddConstraint(Constraint* constraint)
{
	constraints.push_back(constraint);
}

void ParticleSystem::AddCollisionConstraint(Constraint* constraint)
{
	collisionConstraints.push_back(constraint);
}

void ParticleSystem::AccumulateForces()
{
	std::for_each(particles.begin(),particles.end(),std::mem_fun(&Particle::AccumulateForce));
}

void ParticleSystem::Verlet(Scalar time)
{
	for(unsigned int i=0; i<particles.size(); i++)
	{
		Vector3 update = particles[i]->Force() * time * time;
		Vector3 temp = particles[i]->position;
		particles[i]->position += particles[i]->position - particles[i]->oldPosition + update;
		particles[i]->oldPosition = temp;
	}
}

void ParticleSystem::SatisfyConstraints()
{
	int iterations = 5;
	for(; iterations > 0; iterations--)
	{
		for(unsigned int i=0; i<constraints.size(); i++)
		{
			constraints[i]->Satisfy();
		}
	}

	for(iterations = 1; iterations > 0; iterations--)
	{
		for(unsigned int i=0; i<collisionConstraints.size(); i++)
		{
			collisionConstraints[i]->Satisfy();
		}
	}
}

void ParticleSystem::HaltMotion()
{
	SatisfyConstraints();
	for(unsigned int i=0; i<particles.size(); i++)
	{
		particles[i]->oldPosition = particles[i]->position;
	}
}

}
}
