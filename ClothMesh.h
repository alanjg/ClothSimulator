#ifndef CLOTHMESH_H
#define CLOTHMESH_H

#include <vector>
#include "Vector3.h"
#include "IRenderable.h"


namespace ajg
{
	namespace physics
	{
		
		class Particle;
		class Mesh;
		class ConvexMesh;
		class ClothMesh: public IRenderable
		{
		public:
			ClothMesh(const Vector3& upperLeft,const Vector3& upperRight,
							const Vector3& lowerRight,const Vector3& lowerLeft,
							int w,int h);
			void Update(Scalar time);
			void HaltMotion();
			void Render();
		private:

			void AccumulateForces();
			void Verlet(Scalar time);
			void SatisfyConstraints();

			Vector3 CalculateNormalMiddle(int i,int j);
			Vector3 CalculateNormal(int i,int j);

			std::vector<Vector3> particles;
			std::vector<Vector3> oldParticles;
			std::vector<Vector3> normals;
			std::vector<Vector2> texcoords;
			std::vector<Vector3> forces;
			std::vector<Vector3> nails;
			std::vector<unsigned short> indices;
			int width,height;
			float u,v,uv;
			Mesh* mesh;
			Vector3 center;
			Scalar radius;
			Vector3 ul,ur;
		};
	}
}
#endif
