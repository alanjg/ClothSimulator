#include "Constraints.h"
#include "Particle.h"
#include "Mesh.h"
#include "ConvexMesh.h"
#include <iostream>
using namespace std;

namespace ajg {
namespace physics {

DistanceConstraint::DistanceConstraint(Particle& a,Particle& b,Scalar dist):
	particle1(a),particle2(b),distance(dist)
{
}

void SatisfyDistanceConstraint(Vector3& p1,Vector3& p2,Scalar distance,Scalar weight)
{
	Vector3 delta = p2 - p1;
	Scalar length = delta.Magnitude();
	Scalar diff = (length - distance) / length;
	delta *= 0.5f * diff * weight;
	p1 += delta;
	p2 -= delta;
	
}

void DistanceConstraint::Satisfy()
{
	SatisfyDistanceConstraint(particle1.position,particle2.position,distance);
}

NailConstraint::NailConstraint(Particle& p,const Vector3& pt):
	particle(p),point(pt)
{	
}
			
void SatisfyNailConstraint(Vector3& particle,const Vector3& point,Scalar weight)
{
	Vector3 delta = point - particle;
	particle += delta * weight;
}

void NailConstraint::Satisfy()
{
	SatisfyNailConstraint(particle.position,point);
}

void NailConstraint::SetPosition(const Vector3& p)
{
	point = p;
}

BoundingBoxConstraint::BoundingBoxConstraint(Particle& p,const Vector3& vmin,const Vector3& vmax):
	particle(p),max(vmax),min(vmin)
{	
}

void SatisfyBoundingBoxConstraint(Vector3& particle,const Vector3& bmin,const Vector3& bmax,Scalar weight)
{
	if(particle[0] < bmin[0])
		particle[0] = bmin[0];
	if(particle[1] < bmin[1])
		particle[1] = bmin[1];
	if(particle[2] < bmin[2])
		particle[2] = bmin[2];
	if(particle[0] > bmax[0])
		particle[0] = bmax[0];
	if(particle[1] > bmax[1])
		particle[1] = bmax[1];
	if(particle[2] > bmax[2])
		particle[2] = bmax[2];

}

void BoundingBoxConstraint::Satisfy()
{
	SatisfyBoundingBoxConstraint(particle.position,min,max);
}

PlaneConstraint::PlaneConstraint(Particle& p,const Vector3& dir,const Vector3& pt):
	particle(p),normal(dir)
{
	normal.Normalize();
	distance = -normal.Dot(pt);
}

void SatisfyPlaneConstraint(Vector3& particle,const Vector3& normal,Scalar distance,Scalar weight)
{
	//find closest point on plane
	Scalar dist = normal.Dot(particle) + distance;
	particle+= -dist * normal;
}

void PlaneConstraint::Satisfy()
{
	SatisfyPlaneConstraint(particle.position,normal,distance);
}

void SatisfySphereCollideConstraint(Vector3& particle,const Vector3& center,Scalar radius,Scalar weight)
{
	Vector3 delta = particle - center;
	Scalar dist = delta.Normalize();
	
	if(dist < radius)
	{
		particle += (radius - dist) * delta;
	}
}

SphereCollideConstraint::SphereCollideConstraint(Particle& p,const Vector3& c,Scalar r):
	particle(p),center(c),radius(r)
{
}

void SphereCollideConstraint::Satisfy()
{
	SatisfySphereCollideConstraint(particle.position,center,radius);
}


ConvexMeshCollideConstraint::ConvexMeshCollideConstraint(Particle& p,const ConvexMesh& mesh):
	particle(p),convexMesh(mesh)
{
}

void SatisfyConvexMeshCollideConstraint(Vector3& particle,const ConvexMesh& mesh,Scalar weight)
{
	Vector3 point;
	if(mesh.Intersect(particle,point))
	{
		particle = point;
	}
}

void ConvexMeshCollideConstraint::Satisfy()
{
	SatisfyConvexMeshCollideConstraint(particle.position,convexMesh);
}

void SatisfyMeshCollideConstraint(Vector3& particle,const Mesh& mesh,Scalar weight)
{
	Vector3 point;
	if(mesh.Intersect(particle,point))
	{
		particle = point;
	}
}

MeshCollideConstraint::MeshCollideConstraint(Particle& p,const Mesh& themesh):
	particle(p),mesh(themesh)
{
}

void MeshCollideConstraint::Satisfy()
{
	SatisfyMeshCollideConstraint(particle.position,mesh);
}

}
}
