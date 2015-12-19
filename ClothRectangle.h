#ifndef CLOTHRECTANGLE_H
#define CLOTHRECTANGLE_H

#include "Vector3.h"
#include "IRenderable.h"
#include <vector>

namespace ajg
{
	namespace physics
	{
		class ParticleSystem;
		class Particle;
		class NailConstraint;
		class ConvexMesh;
		class ClothRectangle: public IRenderable
		{
		public:
			ClothRectangle(const Vector3& upperLeft,const Vector3& upperRight,
							const Vector3& lowerRight,const Vector3& lowerLeft,
							int w,int h,ParticleSystem& p);
			void Render();
			void SetCorner(const Vector3& point);
		private:
			std::vector<Particle*> particles;
			int width;
			int height;

			void MakeDistanceConstraint(int p1,int p2,ParticleSystem& p);
			Vector3 CalculateNormal(int i,int j);
			NailConstraint* nc;
			ConvexMesh* mesh;
		};
	}
}

#endif
