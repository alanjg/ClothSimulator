#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <vector>
#include "Vector3.h"

namespace ajg
{
	namespace physics
	{
		class Constraint;
		class Particle;
		class ParticleSystem
		{
		public:
			void Update(Scalar time);
			void AddParticle(const Vector3& position,const Vector3& velocity);
			void AddParticle(Particle* particle);
			void AddConstraint(Constraint* constraint);
			void AddCollisionConstraint(Constraint* constraint);

			void HaltMotion();
		private:
			void AccumulateForces();
			void Verlet(Scalar time);
			void SatisfyConstraints();

			std::vector<Particle*> particles;
			std::vector<Constraint*> constraints;
			std::vector<Constraint*> collisionConstraints;
		};

	}
}
#endif
