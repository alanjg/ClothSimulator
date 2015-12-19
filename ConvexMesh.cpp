#include "ConvexMesh.h"
#include <gl/glut.h>
#include <iostream>
namespace ajg {
namespace physics {

ConvexMesh::ConvexMesh(const std::vector<Vector3>& verts,const std::vector<unsigned int>& inds):
	vertices(verts),indexes(inds),planes(inds.size()/3)
{
	for(unsigned int i=0;i<planes.size();i++)
	{
		Vector3 a = vertices[indexes[i*3+2]] - vertices[indexes[i*3+1]];
		Vector3 b = vertices[indexes[i*3+0]] - vertices[indexes[i*3+1]];
		planes[i].normal = a.Cross(b);
		planes[i].normal.Normalize();
		planes[i].distance = -planes[i].normal.Dot(vertices[indexes[i*3]]);
	}
}


bool ConvexMesh::Intersect(const Vector3& point,Vector3& projectedPoint) const
{
	Scalar bestDepth = -999999;
	int bestPlane = -1;
	for(unsigned int i = 0; i < planes.size(); i++)
	{
		Scalar result = point.Dot(planes[i].normal) + planes[i].distance;
		
		result -= 0.1;
		if(result > 0)
			return false;
		
		if(result > bestDepth)
		{
			bestDepth = result;
			bestPlane = i;
		}
	}
	projectedPoint = point - bestDepth * planes[bestPlane].normal;
	return true;
}

void ConvexMesh::Render()
{
//	glDisable(GL_LIGHTING);
//	glDisable(GL_CULL_FACE);

	
//	glVertexPointer(3,GL_FLOAT,0,&vertices[0]);
//	glDrawElements(GL_TRIANGLES,indexes.size(),GL_UNSIGNED_INT,&indexes[0]);

	float white[]={1,1,1,1};
	glMaterialfv(GL_FRONT,GL_DIFFUSE,white);
	
//	immediate mode
	glBegin(GL_TRIANGLES);
	for(unsigned int i=0;i<indexes.size();i++)
	{
		glNormal3fv(planes[i/3].normal.pointer());
		glVertex3fv(vertices[indexes[i]].pointer());
	}
	glEnd();
	
	
//	glEnable(GL_LIGHTING);
//	glEnable(GL_CULL_FACE);
}

}
}
