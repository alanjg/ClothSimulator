#ifndef CONVEXIFIERMESH_H
#define CONVEXIFIERMESH_H

#include <vector>
#include "Vector3.h"
#include "Plane.h"

namespace ajg
{
	namespace physics 
	{

		struct Tri
		{
			int vi[3];
		};

		struct Vert
		{
			std::vector<int> tris;
			Vector3 pos;
		};

		//like a convex mesh, but don't use all planes
		struct PartialConvexMesh
		{
			PartialConvexMesh(const std::vector<Vector3>& verts);
			void AddTriangle(const Tri& tri);
			void AddCapTriangle(const Tri& tri);
			bool CanAdd(const Tri& tri);
			
			const std::vector<Vector3>& vertices;
			std::vector<Tri> triangles;
			std::vector<Tri> phantomTriangles;
		};

		class ConvexifierMesh
		{
		public:
			//construct with an indexed tri list
			ConvexifierMesh(const std::vector<Vector3>& vertices,const std::vector<unsigned int>& indexes);
			//see if the point is in the mesh
			//projectedPoint has the closest point on the surface
			 bool Intersect(const Vector3& point,Vector3& projectedPoint) const;
			 void Render();
		private:
			std::vector<PartialConvexMesh> meshes;
			std::vector<Vector3> vertices;
			
		};

	}
}

#endif
