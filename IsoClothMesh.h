#ifndef ISOCLOTHMESH_H
#define ISOCLOTHMESH_H

#include <vector>
#include "Vector3.h"
#include "IRenderable.h"
namespace ajg
{
	namespace physics
	{
		
		class Particle;
		class ConvexMesh;
		class IsoClothMesh: public IRenderable
		{
		public:
			IsoClothMesh(const Vector3& point,const Vector3& uVec,const Vector3& vVec,int w,int h);
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
			ConvexMesh* mesh;
			Vector3 ul,ur,ll,lr;
		};
	}
}
#endif