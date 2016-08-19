#include "IsoClothMesh.h"
#include <iostream>
#include <algorithm>
#include <functional>
#include "Particle.h"
#include "Constraints.h"
#include "ConvexMesh.h"
#include <gl/glut.h>
#include <cassert>

namespace 
{
	void AddTriangle(std::vector<unsigned int>& v,int a,int b,int c)
	{
		v.push_back(a); v.push_back(b); v.push_back(c);
	}
}

namespace ajg {
namespace physics {

IsoClothMesh::IsoClothMesh(const Vector3& point,const Vector3& uVec,const Vector3& vVec,int w,int h):
	forces(w*h),particles(w*h),oldParticles(w*h),width(w),height(h),ul(point),ur(point + uVec),ll(point + vVec),lr(point + uVec + vVec),nails(w), normals(w*h), texcoords(w*h)
{
	assert(w>2);
	assert(h>1);

	Vector3 du = (uVec)/(w-1.5);
	u = du.Magnitude();
	v = u;
	
	Vector3 dv = vVec;
	dv.Normalize();

//	uv = v * sqrt(3);
	uv = u;

	ll = ul + dv*v*(h-1);
	lr = ll + ur - point; 

	std::cout<<"u: "<<u<<std::endl;
	std::cout<<"v: "<<v<<std::endl;
	std::cout<<"uv: "<<uv<<std::endl;

	std::cout<<"ul: "<<ul<<std::endl;
	std::cout<<"ur: "<<ur<<std::endl;
	std::cout<<"ll: "<<ll<<std::endl;
	std::cout<<"lr: "<<lr<<std::endl;


	//create mesh
	for(int i=0; i < height; i++)
	{
		Scalar alpha = Scalar(i)/(height-1);
		Vector3 left = (1-alpha) * ul + alpha * ll;
		Vector3 right = (1-alpha) * ur + alpha * lr;
		
		oldParticles[i*width] = particles[i*width] = left;
		oldParticles[(i+1)*width-1] = particles[(i+1)*width-1] = right;

		for(int j=1; j < width - 1; j++)
		{
			Scalar offset = (i%2 == 0) ? 0.5f : 0.0f;
			Scalar beta = Scalar(j - offset)/(width - 1);
			Vector3 position = (1 - beta) * left + beta * right;
			particles[i*width + j] = position;
			oldParticles[i*width + j] = position;		
		}
	}

	//set up indices
	int di[]={1,-1};
	int start[]={0,width-1};
	int end[]={width,-1};

	indices.push_back(0);
	for(int i=0; i < height-1; i++)
	{
		int index = i%2;
		indices.push_back((i+1)*width+start[index]);
		for(int j=start[index]+di[index]; j != end[index]; j += di[index])
		{
			indices.push_back(i*width+j);
			indices.push_back((i+1)*width+j);
		}
		indices.push_back((i+1)*width+end[index]-di[index]);
		indices.push_back((i+1)*width+end[index]-di[index]);
	}
	for(int i=0;i<width; i++)
	{
		nails[i] = particles[i];
	}

	std::vector<Vector3> vs;

	Scalar rad = 1.2;
	Scalar x[]={-rad,rad};
	Scalar y[]={-rad,rad};
	Scalar z[]={-rad,rad};

	for(int i=0;i<2;i++)
		for(int j=0;j<2;j++)
			for(int k=0;k<2;k++)
				vs.push_back(Vector3(x[k],y[j],z[i]));

	std::vector<unsigned int> is;
	AddTriangle(is,0,2,1);
	AddTriangle(is,1,2,3);
	AddTriangle(is,4,6,0);
	AddTriangle(is,0,6,2);
	AddTriangle(is,4,7,6);
	AddTriangle(is,4,5,7);
	AddTriangle(is,1,7,5);
	AddTriangle(is,1,3,7);
	AddTriangle(is,2,7,3);
	AddTriangle(is,2,6,7);
	AddTriangle(is,0,1,5);
	AddTriangle(is,0,5,4);
	mesh = new ConvexMesh(vs,is);
}

void IsoClothMesh::Update(Scalar time)
{
	AccumulateForces();
	Verlet(time);
	SatisfyConstraints();
}

void IsoClothMesh::AccumulateForces()
{
	const float dragCoefficient = 0.2;
	static float count = 0;
	count += 0.01;

	for(unsigned int i=0;i<forces.size();i++)
	{
		//gravity
		forces[i] = Vector3(0,-9.8f,0);

		//drag
		forces[i] += -(particles[i] - oldParticles[i]) * dragCoefficient;

		//wind
		forces[i] += (sinf(count) + 1)* Vector3(3,0,0.01);
	}
}

void IsoClothMesh::Verlet(Scalar time)
{
	for(unsigned int i=0; i<particles.size(); i++)
	{
		Vector3 update = forces[i] * time * time;
		Vector3 temp = particles[i];
		particles[i]+= particles[i]- oldParticles[i] + update;
		oldParticles[i] = temp;
	}
}

void IsoClothMesh::SatisfyConstraints()
{
	int iterations = 15;
	for(; iterations > 0; iterations--)
	{
		//satisfy constraints

		for(int i = 0; i < height - 1; i++)
		{
			if(i % 2 == 0)
			{
				SatisfyDistanceConstraint(particles[i*width],particles[i*width+1],u/2);
				SatisfyDistanceConstraint(particles[i*width],particles[(i+1)*width],v);
				for(int j = 1; j < width - 1; j++)
				{	
					SatisfyDistanceConstraint(particles[i*width+j],particles[i*width+j+1],u);
					SatisfyDistanceConstraint(particles[i*width+j],particles[(i+1)*width+j-1],uv);
					SatisfyDistanceConstraint(particles[i*width+j],particles[(i+1)*width+j],uv);
			
					//bend
					if(j > 0)
						SatisfyDistanceConstraint(particles[i*width+j-1],particles[i*width+j+1],2*u);
					else
						SatisfyDistanceConstraint(particles[i*width+j-1],particles[i*width+j+1],1.5*u);
					
					if(i > 0 && i < height - 1)
					{
						
						SatisfyDistanceConstraint(particles[(i-1)*width+j-1],particles[(i+1)*width+j],2*uv);
						SatisfyDistanceConstraint(particles[(i-1)*width+j],particles[(i+1)*width+j-1],2*uv);
					}
				}
				SatisfyDistanceConstraint(particles[(i+1)*width-1],particles[(i+2)*width-2],uv);
				SatisfyDistanceConstraint(particles[(i+1)*width-1],particles[(i+2)*width-1],v);

			}
			else
			{
				SatisfyDistanceConstraint(particles[(i+1)*width-1],particles[(i+1)*width-2],u/2);
				SatisfyDistanceConstraint(particles[(i+1)*width-1],particles[(i+2)*width-1],v);
				for(int j = 0; j < width - 2; j++)
				{	
					SatisfyDistanceConstraint(particles[i*width+j],particles[i*width+j+1],u);
					SatisfyDistanceConstraint(particles[i*width+j],particles[(i+1)*width+j],uv);
					SatisfyDistanceConstraint(particles[i*width+j],particles[(i+1)*width+j+1],uv);
					
					//bend
					if(j < width - 3)
						SatisfyDistanceConstraint(particles[i*width+j],particles[i*width+j+2],2*u);
					else
						SatisfyDistanceConstraint(particles[i*width+j],particles[i*width+j+2],1.5*u);

					if(i > 0 && i < height - 1)
					{
						SatisfyDistanceConstraint(particles[(i-1)*width+j],particles[(i+1)*width+j+1],2*uv);
						SatisfyDistanceConstraint(particles[(i-1)*width+j+1],particles[(i+1)*width+j],2*uv);
					}
				}
				SatisfyDistanceConstraint(particles[(i+1)*width-2],particles[(i+2)*width-2],uv);
				SatisfyDistanceConstraint(particles[(i+1)*width-2],particles[(i+2)*width-1],v);
				
			}
			
		}

		////bend
		//for(int i = 1; i < height - 1; i++)
		//{
		//	for(int j = 1; j < width - 1; j++)
		//	{
		//		SatisfyDistanceConstraint(particles[(i-1)*width + j],particles[(i+1)*width + j],2*v);
		//		SatisfyDistanceConstraint(particles[i*width + j-1],particles[i*width + j+1],2*u);
		//	}
		//}
	
		//collision
		for(int i=0;i<width*height;i++)
			SatisfyConvexMeshCollideConstraint(particles[i],*mesh);

		//nail
//		for(int i=0;i<width;i++)
//			SatisfyNailConstraint(particles[i],nails[i]);

		SatisfyNailConstraint(particles[0],ul);
		SatisfyNailConstraint(particles[width-1],ur);
//		SatisfyNailConstraint(particles[width*(height-1)],ll);
//		SatisfyNailConstraint(particles[width*height-1],lr);

	}
}

void IsoClothMesh::HaltMotion()
{
	SatisfyConstraints();
	for(unsigned int i=0; i<particles.size(); i++)
	{
		oldParticles[i] = particles[i];
	}
}

void IsoClothMesh::Render()
{

	for(int i=1; i < height-1; i++)
	{
		normals[i*width] = CalculateNormal(i,0);
		normals[(i+1)*width-1] = CalculateNormal(i,width-1);
	}

	for(int j=0; j < width; j++)
	{
		normals[j] = CalculateNormal(0,j);
		normals[(height-1)*width+j] = CalculateNormal(height-1,j);
	}
	
	for(int i=1; i < height-1; i++)
	{
		for(int j=1; j < width-1; j++)
		{
			normals[i*width+j] = CalculateNormalMiddle(i,j);
		}
	}

	if(particles.size() > 65535)
	{
		std::cerr << "ERROR, need to change GL_UNSIGNED_SHORT TO GL_UNSIGNED_INT" << std::endl;
		exit(0);
	}
	glVertexPointer(3,GL_FLOAT,0,&particles[0]);
	glNormalPointer(GL_FLOAT,0,&normals[0]);
	glTexCoordPointer(2,GL_FLOAT,0,&texcoords[0]);
	glDrawElements(GL_TRIANGLE_STRIP,GLsizei(indices.size()),GL_UNSIGNED_SHORT,&indices[0]);
	mesh->Render();
	return;

	glBegin(GL_TRIANGLES);
			
	for(int i=1; i < height; i++)
	{
		for(int j=1; j < width; j++)
		{
			Vector3 ul = particles[(i-1)*width + j - 1];
			Vector3 uln = normals[(i-1)*width + j - 1];
			Vector3 ur = particles[(i-1)*width + j];
			Vector3 urn = normals[(i-1)*width + j];
			Vector3 ll = particles[i*width + j - 1];
			Vector3 lln = normals[i*width + j - 1];
			Vector3 lr = particles[i*width + j];
			Vector3 lrn = normals[i*width + j];
		
			glNormal3fv(uln.pointer());
			glVertex3fv(ul.pointer());

			glNormal3fv(lln.pointer());
			glVertex3fv(ll.pointer());

			if((i + j) % 2)
			{
				glNormal3fv(urn.pointer());
				glVertex3fv(ur.pointer());
			}
			else
			{
				glNormal3fv(lrn.pointer());
				glVertex3fv(lr.pointer());
			}

			if((i + j) % 2)
			{
				glNormal3fv(lln.pointer());
				glVertex3fv(ll.pointer());
			}
			else
			{
				glNormal3fv(uln.pointer());
				glVertex3fv(ul.pointer());
			}

			glNormal3fv(lrn.pointer());
			glVertex3fv(lr.pointer());

			glNormal3fv(urn.pointer());
			glVertex3fv(ur.pointer());

		}
	}
	glEnd();
	mesh->Render();
}

//fix this
Vector3 IsoClothMesh::CalculateNormalMiddle(int i,int j)
{
	Vector3 l,r,u,d;
	
	l = particles[i*width + j - 1] - particles[i*width + j];
	r = particles[i*width + j + 1] - particles[i*width + j];
	u = particles[(i-1)*width + j] - particles[i*width + j];
	d = particles[(i+1)*width + j] - particles[i*width + j];
	Vector3 n1 = l.Cross(d);
	Vector3 n2 = r.Cross(u);
	n1.Normalize();
	n2.Normalize();
	n1 = n1 + n2;
	n1.Normalize();
	return n1;
}

Vector3 IsoClothMesh::CalculateNormal(int i,int j)
{
	bool left = true;
	bool up = true;
	bool down = true;
	bool right = true;

	if(j == 0)
		left = false;
	if(j == width - 1)
		right = false;
	if(i == 0)
		up = false;
	if(i == height - 1)
		down = false;

	Vector3 l,r,u,d;
	if(left)
	{
		l = particles[i*width + j - 1] - particles[i*width + j];
	}
	if(right)
	{
		r = particles[i*width + j + 1] - particles[i*width + j];
	}
	if(up)
	{
		u = particles[(i-1)*width + j] - particles[i*width + j];
	}
	if(down)
	{
		d = particles[(i+1)*width + j] - particles[i*width + j];
	}
	Vector3 normal;
	if(left && right && up && down)
	{
		Vector3 n1,n2,n3,n4;
		n1 = l.Cross(d);
		n2 = d.Cross(r);
		n3 = r.Cross(u);
		n4 = u.Cross(l);
		n1.Normalize();
		n2.Normalize();
		n3.Normalize();
		n4.Normalize();
		normal = n1 + n2 + n3 + n4;
	}
	else if(left && up)
	{
		normal = u.Cross(l);
	}
	else if(right && up)
	{
		normal = r.Cross(u);
	}
	else if(right && down)
	{
		normal = d.Cross(r);
	}
	else if(down && left)
	{
		normal = l.Cross(d);
	}
	else
	{
		throw exception("bad normal calc patch");
	}

	normal.Normalize();
	return normal;
}

}
}