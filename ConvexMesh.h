#ifndef CONVEXMESH_H
#define CONVEXMESH_H

#include <vector>
#include "Vector3.h"
#include "Plane.h"

namespace ajg
{
	namespace physics 
	{

		class ConvexMesh
		{
		public:
			//construct with an indexed tri list
			ConvexMesh(const std::vector<Vector3>& vertices,const std::vector<unsigned int>& indexes);
			//see if the point is in the mesh
			//projectedPoint has the closest point on the surface
			 bool Intersect(const Vector3& point,Vector3& projectedPoint) const;
			 void Render();
		private:
			std::vector<Vector3> vertices;
			std::vector<Plane> planes;
			std::vector<unsigned int> indexes;
		};
	}
}

#endif
