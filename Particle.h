#ifndef PARTICLE_H
#define PARTICLE_H
#include "Vector3.h"
#include <vector>

namespace ajg
{
	namespace physics
	{
		class IForce;
		class Particle
		{
		public:
			Particle(const Vector3& pos,const Vector3& vel)
			{
				position = pos;
				oldPosition = position - vel;
				force = Vector3::Zero();
			}
			Particle()
			{
				position = oldPosition = force = Vector3::Zero();
			}
			Vector3 normal;
			Vector3 position;
			Vector3 oldPosition;
			void AccumulateForce();
			void AddForce(IForce* f);
			Vector3 Force() const
			{
				return force; 
			}
		private:
			std::vector<IForce*> forces;
			Vector3 force;
		};

	}
}

#endif
