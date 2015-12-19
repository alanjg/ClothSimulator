#include "ClothRectangle.h"
#include "Particle.h"
#include "ParticleSystem.h"
#include "IConstraint.h"
#include "Forces.h"
#include "Constraints.h"
#include "ConvexMesh.h"
#include <gl/glut.h>
#include <vector>
#include <exception>

namespace ajg {
namespace physics {

namespace
{

	void AddTriangle(std::vector<unsigned int>& v,int a,int b,int c)
	{
		v.push_back(a); v.push_back(b); v.push_back(c);
	}
}

ClothRectangle::ClothRectangle(const Vector3& upperLeft,const Vector3& upperRight,
							   const Vector3& lowerRight,const Vector3& lowerLeft,
							   int w,int h,ParticleSystem& p):
	particles(w*h),width(w),height(h)
{
	Vector3 center = (lowerRight + lowerLeft) / 2;
	Scalar radius = (upperLeft - lowerLeft).Magnitude() / 2;
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
			particles[i*width + j] = new Particle(position,Vector3::Zero());
			p.AddParticle(particles[i*width + j]);
			particles[i*width + j]->AddForce(new GravityForce());
			particles[i*width + j]->AddForce(new WindForce(Vector3(3,0,0.01)));
			particles[i*width + j]->AddForce(new DragForce(*particles[i*width+j],0.2));
			
		}
	}
	
	//add constraints
	
	
	//left side
	for(int i = 1; i < height; i++)
	{
		MakeDistanceConstraint(i*width,(i-1)*width,p);
	}
	
	//top side
	for(int j = 1; j < width; j++)
	{
		MakeDistanceConstraint(j,j-1,p);
	}
	
	//fill out mesh
	for(int i = 1; i < height; i++)
	{
		for(int j = 1; j < width; j++)
		{
			//bottom
			MakeDistanceConstraint(i*width + j,i*width + j-1,p);

			//right
			MakeDistanceConstraint(i*width + j,(i-1)*width + j,p);
			
			//diagonals
//			if(i + j % 2)
				MakeDistanceConstraint(i*width + j,(i-1)*width + j - 1,p);
//			else
				MakeDistanceConstraint((i-1)*width + j,i*width + j - 1,p);

			//go over one
			if(i < height-1)
			{
				MakeDistanceConstraint((i+1)*width + j,(i-1)*width + j,p);
			}
			if(j < width-1)
			{
				MakeDistanceConstraint(i*width + j+1,i*width + j-1,p);
			}
		}
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

	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
//			p.AddConstraint(new SphereCollideConstraint(*particles[i*width + j],center,radius));
			p.AddCollisionConstraint(new ConvexMeshCollideConstraint(*particles[i*width + j],*mesh));		
		}
	}
		//upper left is pegged
	p.AddConstraint(nc = new NailConstraint(*particles[0],particles[0]->position));
//	p.AddConstraint(new PlaneConstraint(*particles[0],Vector3(0,1,0),particles[0]->position));

	//lower left is pegged
//	p.AddConstraint(new NailConstraint(*particles[width*(height-1)],particles[width*(height-1)]->position));

	p.AddConstraint(new NailConstraint(*particles[width-1],particles[width-1]->position));
//	p.AddConstraint(new PlaneConstraint(*particles[width-1],Vector3(0,1,0),particles[width-1]->position));

//	p.AddConstraint(new NailConstraint(*particles[(height-1)*(width)],particles[(height-1)*(width)]->position));
//	p.AddConstraint(new NailConstraint(*particles[height*width-1],particles[height*width-1]->position));

}

void ClothRectangle::SetCorner(const Vector3& point)
{
	nc->SetPosition(point);
}

void ClothRectangle::Render()
{
#if 0
	GLUnurbsObj* nobj = gluNewNurbsRenderer();
	std::vector<Vector3> verts;
	std::vector<Vector3> norms;
	
	std::vector<float> sknot,tknot;
	for(int i=0; i < height; i++)
	{
		for(int j=0; j < width; j++)
		{
			verts.push_back(particles[i*width+j]->position);
			norms.push_back(CalculateNormal(i,j));			
		}
		sknot.push_back(i);
		tknot.push_back(i);
	}
	sknot.push_back(height);
	sknot.push_back(height+1);
	sknot.push_back(height+2);
	sknot.push_back(height+3);
	
	tknot.push_back(height);
	tknot.push_back(height+1);
	tknot.push_back(height+2);
	tknot.push_back(height+3);

	gluBeginSurface(nobj); 
    gluNurbsSurface(nobj,(width+4),&sknot[0],(height+4),&tknot[0],3,3*width,&verts[0][0],4,4,GL_MAP2_VERTEX_3); 
    gluNurbsSurface(nobj,(width+4),&sknot[0],(height+4),&tknot[0],3,3*width,&norms[0][0],4,4, GL_MAP2_NORMAL); 
    
	gluEndSurface(nobj); 

	gluDeleteNurbsRenderer(nobj);
#else
#if 1
	glBegin(GL_TRIANGLES);
			
	for(int i=1; i < height; i++)
	{
		for(int j=1; j < width; j++)
		{
			Vector3 ul = particles[(i-1)*width + j - 1]->position;
			Vector3 uln = CalculateNormal(i - 1,j - 1);
			Vector3 ur = particles[(i-1)*width + j]->position;
			Vector3 urn = CalculateNormal(i - 1,j);
			Vector3 ll = particles[i*width + j - 1]->position;
			Vector3 lln = CalculateNormal(i,j - 1);
			Vector3 lr = particles[i*width + j]->position;
			Vector3 lrn = CalculateNormal(i,j);
		
			Vector3 mid = (ul + ur + ll + lr) / 4;
			Vector3 mn = uln + urn + lln + lrn;
			mn.Normalize();
			
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
#else
	for(int i=1; i < height; i++)
	{
		glBegin(GL_TRIANGLE_STRIP);
		
		for(int j=0; j < width; j++)
		{
			Vector3 normal;
			normal = CalculateNormal(i-1,j);
			glNormal3fv(normal.pointer());
			glVertex3fv(particles[(i-1)*width + j]->position.pointer());

			normal = CalculateNormal(i,j);
			glNormal3fv(normal.pointer());
			glVertex3fv(particles[i*width + j]->position.pointer());
		}
	
		glEnd();
	}
	
	//draw the back
	for(int i=1; i < height; i++)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for(int j=0; j < width; j++)
		{
			Vector3 normal;
			normal = -CalculateNormal(i,j);
			glNormal3fv(normal.pointer());
			glVertex3fv(particles[i*width + j]->position.pointer());

			normal = -CalculateNormal(i-1,j);
			glNormal3fv(normal.pointer());
			glVertex3fv(particles[(i-1)*width + j]->position.pointer());
		}
		glEnd();
	}
#endif
#endif

	
	mesh->Render();
}

Vector3 ClothRectangle::CalculateNormal(int i,int j)
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
		l = particles[i*width + j - 1]->position - particles[i*width + j]->position;
	}
	if(right)
	{
		r = particles[i*width + j + 1]->position - particles[i*width + j]->position;
	}
	if(up)
	{
		u = particles[(i-1)*width + j]->position - particles[i*width + j]->position;
	}
	if(down)
	{
		d = particles[(i+1)*width + j]->position - particles[i*width + j]->position;
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

void ClothRectangle::MakeDistanceConstraint(int p1,int p2,ParticleSystem& p)
{
	Scalar dist = (particles[p1]->position - particles[p2]->position).Magnitude();
	p.AddConstraint(new DistanceConstraint(*particles[p1],*particles[p2],dist));
}

}
}
