#ifndef FORCES_H
#define FORCES_H
#include "IForce.h"

namespace ajg 
{
	namespace physics
	{
		class Particle;

		class GravityForce: public IForce
		{
		public:
			Vector3 Apply();
		private:
		};

		class DragForce: public IForce
		{
		public:
			DragForce(Particle& p,Scalar drag);
			Vector3 Apply();
		private:
			Particle& particle;
			Scalar dragCoefficient;
		};

		class SpringForce: public IForce
		{
		public:
			SpringForce(Particle& p1,Particle& p2,Scalar k);
			Vector3 Apply();
		private:
			Particle& particle;
			Particle& other;
			Scalar stiffness;
		};

		class WindForce: public IForce
		{
		public:
			WindForce(const Vector3& dir);
			Vector3 Apply();
		private:
			const Vector3 direction;
			Scalar count;
		};
	}
}
#endif
