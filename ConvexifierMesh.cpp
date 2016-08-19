#include "ConvexifierMesh.h"
#include <gl/glut.h>
#include <iostream>
#include <queue>

namespace ajg {
namespace physics {

PartialConvexMesh::PartialConvexMesh(const std::vector<Vector3>& verts):
	vertices(verts)
{

}

void PartialConvexMesh::AddTriangle(const Tri& tri)
{
	triangles.push_back(tri);
}


void PartialConvexMesh::AddCapTriangle(const Tri& tri)
{
	phantomTriangles.push_back(tri);
}

bool PartialConvexMesh::CanAdd(const Tri& tri)
{
	for(unsigned int i = 0; i < triangles.size(); i++)
	{
		for(int j=0;j<3;j++)
		{
			Vector3 vertex = vertices[tri.vi[j]];
			//if vertex is on the wrong side of triangles[i], return false;
		}
	}
	return true;
}

ConvexifierMesh::ConvexifierMesh(const std::vector<Vector3>& vert,const std::vector<unsigned int>& indices):
	vertices(vert)
{
	std::vector<Tri> tris(indices.size()/3);
	std::vector<Vert> verts(vertices.size());
	for(unsigned int i=0;i<tris.size();i++)
	{
		for(int j=0;j<3;j++)
		{
			tris[i].vi[j] = indices[i*3+j];
			verts[indices[i*3+j]].tris.push_back(i);
		}
	}
	for(unsigned int i=0;i<verts.size();i++)
	{
		verts[i].pos = vertices[i];
	}
	std::vector<int> faces(tris.size());
	for(unsigned int i = 0; i < faces.size(); i++)
	{
		faces[i] = i;
	}
	std::vector<bool> used(faces.size(),false);
	while(!faces.empty())
	{
		PartialConvexMesh mesh(vertices);
		
		std::queue<int> faceQueue;
		faceQueue.push(faces.front());

		while(!faceQueue.empty())
		{
			int face = faceQueue.front();
			faceQueue.pop();

			if(used[face])
				continue;

			if(mesh.CanAdd(tris[face]))
				mesh.AddTriangle(tris[face]);

			faces.erase(faces.begin());
			used[face] = true;

			for(int i=0;i < 3;i++)
			{
				for(unsigned int j = 0; j < verts[tris[face].vi[i]].tris.size(); j++)
				{
					int poly = verts[tris[face].vi[i]].tris[j];
					if(used[poly])
						continue;
					faceQueue.push(poly);
				}
			}
		}
		//face queue is empty, make this mesh convex, and add new polys
		//look at code i have at home to do this
		//in my old 3d engine
	}
}


bool ConvexifierMesh::Intersect(const Vector3& point,Vector3& projectedPoint) const
{
	return true;
}

void ConvexifierMesh::Render()
{

}

}
}
