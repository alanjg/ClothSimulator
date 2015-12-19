#include "Mesh.h"
#include <gl/glut.h>
#include <iostream>
#include <algorithm>
#include <exception>

namespace ajg {
namespace physics {

Edge::Edge()
{

}

Edge::Edge(const Vector3& point1,const Vector3& point2,const Plane& plane)
{
	edge = point2 - point1;
	Vector3 normal = edge.Cross(plane.normal);

	xi = 0;
	if(fabsf(normal[1]) > fabsf(normal[0]))
		xi = 1;
	if(fabsf(normal[2]) > fabsf(normal[xi]))
		xi = 2;

	yi = 0;
	if(fabsf(edge[1]) > fabsf(edge[0]))
		yi = 1;
	if(fabsf(edge[2]) > fabsf(edge[yi]))
		yi = 2;

	if(xi == yi)
	{
		if(fabs(normal[(xi+1)%3]) > 0)
		{
			xi = (xi+1)%3;
		}
		else if(fabs(normal[(xi+2)%3]) > 0)
		{
			xi = (xi+2)%3;
		}
		else if(fabs(edge[(yi+1)%3]) > 0)
		{
			yi = (yi+1)%3;
		}
		else if(fabs(edge[(yi+2)%3]) > 0)
		{
			yi = (yi+2)%3;
		}
		else
		{
			//std::cerr<<normal<<" "<<edge<<std::endl;
			//throw std::exception("degenernormalte trinormalngle!");
		}
	}

	corner = point1;
	inverseEdgeCrossNormal = 1.0 / (edge[yi]*normal[xi] - edge[xi]*normal[yi]);
	cornerCrossNormal = corner[xi]*normal[yi] - corner[yi]*normal[xi];
	normala = normal[xi];
	normalb = normal[yi];
	normala *= inverseEdgeCrossNormal;
	normalb *= inverseEdgeCrossNormal;
	cornerCrossNormal *= inverseEdgeCrossNormal;
	
}

Scalar Edge::DistanceSquared(const Vector3& point,const Vector3& projectedPoint) const
{
//	std::cerr<<"PLD corner:"<<corner<<" point:"<<point<<" edge vec:"<<edge<<" normal:"<<normal<<std::endl;
	//corner + edge * beta + normal * alpha = point
	//solve for beta
	
	Scalar beta = projectedPoint[yi]*normala - projectedPoint[xi]*normalb + cornerCrossNormal;

//	std::cerr << "alpha = " << alpha << std::endl;
//	std::cerr << "beta = " << beta << std::endl;
//	std::cerr << "closest point: "<<(corner + edge * beta)<<std::endl;

	Vector3 pt(corner);
	if(beta >= 1)
	{
		pt += edge;
	}
	else if( beta > 0)
	{
		pt += beta * edge;
	}
	return (point-pt).MagnitudeSquared();
}	

Triangle::Triangle()
{
}

Triangle::Triangle(const Vector3& point1,const Vector3& point2,const Vector3& point3)
{
	Vector3 a = point3 - point2;
	Vector3 b = point1 - point2;
	p = point2;
	up = point3;
	vp = point1;
	
	plane.normal = a.Cross(b);
	plane.normal.Normalize();
	plane.distance = -plane.normal.Dot(point1);

	ai = 0;
	if(fabsf(a[1]) > fabsf(a[ai]))
		ai = 1;
	if(fabsf(a[2]) > fabsf(a[ai]))
		ai = 2;

	bi = 0;
	if(fabsf(b[1]) > fabsf(b[bi]))
		bi = 1;
	if(fabsf(b[2]) > fabsf(b[bi]))
		bi = 2;
	if(ai == bi)
	{
		if(fabs(a[(ai+1)%3]) > 0)
		{
			ai = (ai+1)%3;
		}
		else if(fabs(a[(ai+2)%3]) > 0)
		{
			ai = (ai+2)%3;
		}
		else if(fabs(b[(bi+1)%3]) > 0)
		{
			bi = (bi+1)%3;
		}
		else if(fabs(b[(bi+2)%3]) > 0)
		{
			bi = (bi+2)%3;
		}
		else
		{
			//throw std::exception("degenerate triangle!");
		}
	}

	edges[0] = Edge(up,vp,plane);
	edges[1] = Edge(p,vp,plane);
	edges[2] = Edge(p,up,plane);
	
	inverseACrossB = 1.0 / (a[ai]*b[bi] - b[ai]*a[bi]);
	bb = b[bi];
	ba = b[ai];
	ab = a[bi];
	bbInverse = 1.0 / bb;

	bb *= inverseACrossB;
	ba *= inverseACrossB;
}

bool Triangle::DistanceSquared(const Vector3& point,Scalar& distance) const
{
	Vector3 projectedPoint = point - (point.Dot(plane.normal) + plane.distance) * plane.normal;
//	std::cerr<<"point:"<<point<<std::endl;
//	std::cerr<<"projpoint:"<<projectedPoint<<std::endl;
	Vector3 pt = projectedPoint - p;

	Scalar u = pt[ai]*bb - ba*pt[bi];
	Scalar v = (pt[bi] - ab*u) * bbInverse;
//	std::cerr<<"a:"<<a<<" b:"<<b<<std::endl;
	Scalar dist = plane.normal.Dot(point) + plane.distance;
	Scalar sign = dist < 0 ? -1 : 1;

	int edge1 = 1;
	int edge2 = 2;
	if(u >= 0)
		edge2 = 0;
	if(v >= 0)
		edge1 = 0;

	if(edge1 + edge2 == 0)
	{
		if(u + v <= 1)
		{
			//point is in triangle projection, return normal distance
			distance = sign * dist * dist;
			return true;
		}
		else
		{
			distance = sign * edges[0].DistanceSquared(point,projectedPoint);
			return false;
		}
	}
	else
	{
		Scalar dist1 = edges[edge1].DistanceSquared(point,projectedPoint);
		Scalar dist2 = edges[edge2].DistanceSquared(point,projectedPoint);
		distance = sign * std::min(dist1,dist2);
		return false;
	}
}

Mesh::Mesh(const std::vector<Vector3>& verts,const std::vector<unsigned int>& inds):
	vertices(verts),indexes(inds),triangles(inds.size()/3)
{
	for(unsigned int i=0;i<triangles.size();i++)
	{
		triangles[i] = Triangle(vertices[indexes[i*3+0]],vertices[indexes[i*3+1]],vertices[indexes[i*3+2]]);
	}
}

//point is intersecting if it's behind the closest plane 
bool Mesh::Intersect(const Vector3& point,Vector3& projectedPoint) const
{
	Scalar depthVal;
	bool inTri = triangles[0].DistanceSquared(point,depthVal);
	Scalar bestDepth = fabsf(depthVal);
	int bestTriangle = 0;
	for(unsigned int i = 1; i < triangles.size(); i++)
	{
		Scalar result;
		bool in = triangles[i].DistanceSquared(point,result);
	
		Scalar res = fabsf(result);
		if(res < bestDepth)
		{
			bestDepth = res;
			depthVal = result;
			bestTriangle = i;
			inTri = in;
		}
	}

	if(depthVal > 0 || !inTri) 
		return false;

//	std::cerr<<"old point:"<<point<<std::endl;
//	std::cerr<<"depthval:"<<depthVal<<" tri:"<<bestTriangle<<std::endl;
	projectedPoint = point + std::sqrt(bestDepth) * triangles[bestTriangle].plane.normal;
//	std::cerr<<"projpointfinal:"<<projectedPoint<<std::endl;
//	char c;
//	std::cin>>c;
	return true;
}

void Mesh::Render()
{

	float white[]={1,1,1,1};
	glMaterialfv(GL_FRONT,GL_DIFFUSE,white);
	glPushMatrix();
	
	Scalar scaleFactor = 0.9;

	glScalef(scaleFactor,scaleFactor,scaleFactor);
//	immediate mode
	glBegin(GL_TRIANGLES);
	for(unsigned int i=0;i<indexes.size();i++)
	{
		glNormal3fv(triangles[i/3].plane.normal.pointer());
		glVertex3fv(vertices[indexes[i]].pointer());
	}
	glEnd();
	glPopMatrix();
	
}

}
}
