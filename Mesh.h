#ifndef MESH_H
#define MESH_H

#include <vector>
#include "Vector3.h"
#include <algorithm>
#include "Plane.h"

namespace ajg
{
	namespace physics 
	{
		struct Edge
		{
			int xi,yi;
			Vector3 corner;
			Vector3 edge;
			Scalar normala,normalb;
			Scalar inverseEdgeCrossNormal;
			Scalar cornerCrossNormal;
			Edge();
			Edge(const Vector3& point1,const Vector3& point2,const Plane& plane);
			Scalar DistanceSquared(const Vector3& point,const Vector3& projectedPoint) const;
		};

		struct Triangle
		{
			Triangle();
			Triangle(const Vector3& point1,const Vector3& point2,const Vector3& point3);
			//p is the corner, a and b are basis functions
			Vector3 p;
//			,a,b;

			Scalar bb,ba,ab,bbInverse;
			//up is the u point, vp is the v point
			Vector3 up,vp;
			//index into largest component of basis function
			int ai,bi;

			Scalar inverseACrossB;
			Edge edges[3];

			Plane plane;
			
			//returns true if its projection is in the triangle
			bool DistanceSquared(const Vector3& point,Scalar& distance) const;
		};

		class Mesh
		{
		public:
			//construct with an indexed tri list
			Mesh(const std::vector<Vector3>& vertices,const std::vector<unsigned int>& indexes);
			//see if the point is in the mesh
			//projectedPoint has the closest point on the surface
			 bool Intersect(const Vector3& point,Vector3& projectedPoint) const;
			 void Render();
		private:
			std::vector<Vector3> vertices;
			std::vector<Triangle> triangles;
			std::vector<unsigned int> indexes;
		};
	}
}

#endif
