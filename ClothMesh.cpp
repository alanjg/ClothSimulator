#include "ClothMesh.h"
#include <iostream>
#include <algorithm>
#include <functional>
#include "Particle.h"
#include "Constraints.h"
#include "Mesh.h"
#include "ConvexMesh.h"
#include <gl/glut.h>
#include <exception>

namespace 
{
	using ajg::physics::Vector3;
	void AddTriangle(std::vector<unsigned int>& v,int a,int b,int c)
	{
		v.push_back(a); v.push_back(b); v.push_back(c);
	}
	void DrawTriangle(const Vector3& n1,const Vector3& p1,const Vector3& n2,const Vector3& p2,const Vector3& n3,const Vector3& p3,int sub,const Vector3& t1,const Vector3& t2,const Vector3& t3)
	{
		if(sub == 0)
		{
			glTexCoord2fv(t1.pointer());
			glNormal3fv(n1.pointer());
			glVertex3fv(p1.pointer());

			glTexCoord2fv(t2.pointer());
			glNormal3fv(n2.pointer());
			glVertex3fv(p2.pointer());

			glTexCoord2fv(t3.pointer());
			glNormal3fv(n3.pointer());
			glVertex3fv(p3.pointer());
			return;
		}
		Vector3 n4 = (n1 + n2);
		n4.Normalize();
		Vector3 n5 = (n1+n3);
		n5.Normalize();
		Vector3 n6 = (n2 + n3);
		n6.Normalize();
		Vector3 p4 = (p1+p2)/2;
		Vector3 p5 = (p1+p3)/2;
		Vector3 p6 = (p2+p3)/2;
		Vector3 t4 = (t1+t2)/2;
		Vector3 t5 = (t1+t3)/2;
		Vector3 t6 = (t2+t3)/2;
//		DrawTriangle(n1,p1,n4,p4,n5,p5,sub-1,t4,t5,t6);
//		DrawTriangle(n2,p2,n6,p6,n4,p4,sub-1,t4,t5,t6);
//		DrawTriangle(n3,p3,n5,p5,n6,p6,sub-1,t4,t5,t6);
//		DrawTriangle(n4,p4,n6,p6,n5,p5,sub-1,t4,t5,t6);
		DrawTriangle(n1,p1,n4,p4,n5,p5,sub-1,t1,t4,t5);
		DrawTriangle(n2,p2,n6,p6,n4,p4,sub-1,t2,t6,t4);
		DrawTriangle(n3,p3,n5,p5,n6,p6,sub-1,t3,t5,t6);
		DrawTriangle(n4,p4,n6,p6,n5,p5,sub-1,t4,t6,t5);

	}
}

namespace ajg {
namespace physics {

	int blah = 0;

ClothMesh::ClothMesh(const Vector3& upperLeft,const Vector3& upperRight,
						const Vector3& lowerRight,const Vector3& lowerLeft,
						int w,int h):
	forces(w*h),particles(w*h),oldParticles(w*h),width(w),height(h),ul(upperLeft),ur(upperRight),nails(w), normals(w*h), texcoords(w*h)
{
	blah = 1;
	center = (lowerRight + lowerLeft) / 2;
	radius = (upperLeft - lowerLeft).Magnitude() / 2;
	center = Vector3::Zero();
	radius = 1.2;

	//create mesh
	for(int i=0; i < height; i++)
	{
		Scalar alpha = Scalar(i)/(height-1);
		Vector3 left = (1-alpha) * upperLeft + alpha * lowerLeft;
		Vector3 right = (1-alpha) * upperRight + alpha * lowerRight;
		for(int j=0; j < width; j++)
		{
			Scalar beta = Scalar(j)/(width - 1);
			Vector3 position = (1 - beta) * left + beta * right;
			particles[i*width + j] = position;
			oldParticles[i*width + j] = position;		
		} 
		
		if(i == height-1) continue;
		//set up indices
		int di[]={1,-1};
		int start[]={0,width-1};
		int end[]={width,-1};
		int index = i%2;
		for(int j=start[index]; j != end[index]; j += di[index])
		{
			indices.push_back(i*width+j);
			indices.push_back((i+1)*width+j);
		}
		indices.push_back((i+1)*width+end[index]-di[index]);
	}
	for(int i=0;i<width; i++)
	{
		nails[i] = particles[i];
	}

	u = (particles[1]-particles[0]).Magnitude();
	v = (particles[width]-particles[0]).Magnitude();
	uv = sqrt(u*u+v*v);

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
	mesh = new Mesh(vs,is);
}

void ClothMesh::Update(Scalar time)
{
	AccumulateForces();
	Verlet(time);
	SatisfyConstraints();
}

void ClothMesh::AccumulateForces()
{
	const float dragCoefficient = 0.8;
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

void ClothMesh::Verlet(Scalar time)
{
	for(unsigned int i=0; i<particles.size(); i++)
	{
		Vector3 update = forces[i] * time * time;
		Vector3 temp = particles[i];
		particles[i]+= particles[i]- oldParticles[i] + update;
		oldParticles[i] = temp;
	}
}

void ClothMesh::SatisfyConstraints()
{
	int iterations = 5;
	float weights[]={1.0f,1.0f,0.9f,0.85f,0.8f,0.75f,0.7f,0.5f};
	for(; iterations > 0; iterations--)
	{
		Scalar weight = weights[iterations];
		weight = 1.0;
		//satisfy constraints

		//left side
		for(int i = 1; i < height; i++)
		{
			SatisfyDistanceConstraint(particles[i*width],particles[(i-1)*width],v,weight);
		}

		//top side
		for(int j = 1; j < width; j++)
		{
			SatisfyDistanceConstraint(particles[j],particles[j-1],u,weight);
		}

		//bend
		for(int i = 1; i < height - 1; i++)
		{
			for(int j = 1; j < width - 1; j++)
			{
				SatisfyDistanceConstraint(particles[(i-1)*width + j],particles[(i+1)*width + j],2*v,weight);
				SatisfyDistanceConstraint(particles[i*width + j-1],particles[i*width + j+1],2*u,weight);
			}
		}
/*
		//diagonal bend
		for(int i = 0; i < height; i++)
		{
			for(int j = 0; j < width; j++)
			{
				//upper left
				if(i > 1 && j > 1)
					SatisfyDistanceConstraint(particles[(i-2)*width + j-2],particles[(i)*width + j],2*uv,weight);
				//upper right
				if(i > 1 && j < width - 2)
					SatisfyDistanceConstraint(particles[(i-2)*width + j+2],particles[i*width + j],2*uv,weight);
			}
		}
*/
		//distance
		for(int i = 1; i < height; i++)
		//for(int i = height-1; i >= 1; i--)
		{
			for(int j = 1; j < width; j++)
			//for(int j = width - 1; j >= 1; j--)
			{
				//bottom
				SatisfyDistanceConstraint(particles[i*width + j],particles[i*width + j-1],u,weight);

				//right
				SatisfyDistanceConstraint(particles[i*width + j],particles[(i-1)*width + j],v,weight);
				
				//diagonals
				SatisfyDistanceConstraint(particles[i*width + j],particles[(i-1)*width + j - 1],uv,weight);
				SatisfyDistanceConstraint(particles[(i-1)*width + j],particles[i*width + j - 1],uv,weight);

				//nail
				//	for(int i=0;i<width;i++)
				//		SatisfyNailConstraint(particles[i],nails[i],weight);

				SatisfyNailConstraint(particles[0],ul,1.0);
				SatisfyNailConstraint(particles[width-1],ur,1.0);
			}
		}
	}
	for(iterations=2; iterations >= 0; iterations--)
	{
		Scalar weight = 1.0;

		//collision
		for(int i=0;i<width*height;i++)
		{
			//SatisfyPlaneConstraint(particles[i],Vector3(0,0,1),0);
			SatisfyMeshCollideConstraint(particles[i],*mesh,weight);

//			SatisfySphereCollideConstraint(particles[i],center+Vector3(0.8,0,0), 1.2,weight);
//			SatisfySphereCollideConstraint(particles[i],center+Vector3(-0.8,0,0), 1.2,weight);
//			SatisfySphereCollideConstraint(particles[i],center, 1.4,weight);
		}


	}
//	center -= Vector3(0,-0.01,0);
	
}

void ClothMesh::HaltMotion()
{
	SatisfyConstraints();
	for(unsigned int i=0; i<particles.size(); i++)
	{
		oldParticles[i] = particles[i];
	}
}

void ClothMesh::Render()
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
	//glVertexPointer(3,GL_FLOAT,0,&particles[0]);
	//glNormalPointer(GL_FLOAT,0,&normals[0]);
	//glTexCoordPointer(2,GL_FLOAT,0,&texcoords[0]);
	//glDrawElements(GL_TRIANGLE_STRIP,GLsizei(indices.size()),GL_UNSIGNED_SHORT,&indices[0]);
	//mesh->Render();
	//return;

	int divs = 1;
	glBegin(GL_TRIANGLES);
			
	for(int i=1; i < height; i++)
	{
		for(int j=1; j < width; j++)
		{
			Vector3 ul = particles[(i-1)*width + j - 1];
			Vector3 uln = normals[(i-1)*width + j - 1];
			//Vector3 ult((i-1)/float(height),(j-1)/float(width),0);
			Vector3 ult((j-1)/float(width-1),(i-1)/float(height-1),0);
			Vector3 ur = particles[(i-1)*width + j];
			Vector3 urn = normals[(i-1)*width + j];
			//Vector3 urt((i-1)/float(height),(j)/float(width),0);
			Vector3 urt((j)/float(width-1),(i-1)/float(height-1),0);
			Vector3 ll = particles[i*width + j - 1];
			Vector3 lln = normals[i*width + j - 1];
			//Vector3 llt((i)/float(height),(j-1)/float(width),0);
			Vector3 llt((j-1)/float(width-1),(i)/float(height-1),0);
			Vector3 lr = particles[i*width + j];
			Vector3 lrn = normals[i*width + j];
			//Vector3 lrt((i)/float(height),(j)/float(width),0);
			Vector3 lrt((j)/float(width-1),(i)/float(height-1),0);
			
			Vector3 mm = (ul + ur + ll + lr)/4;
			Vector3 mmn = (uln + urn + lln + lrn);
			Vector3 mmt = (ult + urt + llt + lrt)/4;
			mmn.Normalize();

			DrawTriangle(uln,ul,lln,ll,mmn,mm,divs,ult,llt,mmt);
			DrawTriangle(lln,ll,lrn,lr,mmn,mm,divs,llt,lrt,mmt);
			DrawTriangle(lrn,lr,urn,ur,mmn,mm,divs,lrt,urt,mmt);
			DrawTriangle(urn,ur,uln,ul,mmn,mm,divs,urt,ult,mmt);
			continue;

			if((i+j)%2)
			{
				DrawTriangle(uln,ul,lln,ll,urn,ur,divs,ult,llt,urt);
				DrawTriangle(lln,ll,lrn,lr,urn,ur,divs,llt,lrt,urt);
			}
			else
			{
				DrawTriangle(uln,ul,lln,ll,lrn,lr,divs,ult,llt,lrt);
				DrawTriangle(uln,ul,lrn,lr,urn,ur,divs,ult,lrt,urt);
			}
			continue;

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
//	glTranslatef(-center[0],-center[1],-center[2]);
//	glutSolidSphere(0.8,20,20);
//	mesh->Render();
}

Vector3 ClothMesh::CalculateNormalMiddle(int i,int j)
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

Vector3 ClothMesh::CalculateNormal(int i,int j)
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
		//throw std::exception("bad normal calc patch");
	}

	normal.Normalize();
	return normal;
}

}
}
