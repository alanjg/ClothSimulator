#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "IConstraint.h"
#include "Vector3.h"

namespace ajg
{
	namespace physics
	{
		class Particle;
		class ConvexMesh;
		class Mesh;

		void SatisfySphereCollideConstraint(Vector3& particle,const Vector3& center,Scalar radius,Scalar weight = 1.0);
		void SatisfyDistanceConstraint(Vector3& p1,Vector3& p2,Scalar distance,Scalar weight = 1.0);

		void SatisfyNailConstraint(Vector3& particle,const Vector3& point,Scalar weight = 1.0);
		void SatisfyBoundingBoxConstraint(Vector3& particle,const Vector3& bmin,const Vector3& bmax,Scalar weight = 1.0);
		void SatisfyPlaneConstraint(Vector3& particle,const Vector3& normal,Scalar distance,Scalar weight = 1.0);
		void SatisfyConvexMeshCollideConstraint(Vector3& particle,const ConvexMesh& mesh,Scalar weight = 1.0);
		void SatisfyMeshCollideConstraint(Vector3& particle,const Mesh& mesh,Scalar weight = 1.0);

		class DistanceConstraint: public Constraint
		{
		public:
			DistanceConstraint(Particle& a,Particle& b,Scalar dist);
			virtual void Satisfy();
		private:
			Particle& particle1;
			Particle& particle2;
			Scalar distance;
		};

		class NailConstraint: public Constraint
		{
		public:
			NailConstraint(Particle& p,const Vector3& pt);
			virtual void Satisfy();
			void SetPosition(const Vector3& p);
		private:
			Particle& particle;
			Vector3 point;
		};

		class BoundingBoxConstraint: public Constraint
		{
		public:
			BoundingBoxConstraint(Particle& p,const Vector3& vmin,const Vector3& vmax);
			virtual void Satisfy();
		private:
			Particle& particle;
			Vector3 min;
			Vector3 max;
		};

		class PlaneConstraint: public Constraint
		{
		public:
			PlaneConstraint(Particle& p,const Vector3& dir,const Vector3& pt);
			virtual void Satisfy();
		private:
			Particle& particle;
			Vector3 normal;
			Scalar distance;
		};

		class SphereCollideConstraint: public Constraint
		{
		public:
			SphereCollideConstraint(Particle& p,const Vector3& c,Scalar r);
			virtual void Satisfy();
		private:
			Particle& particle;
			Vector3 center;
			Scalar radius;
		};

		class ConvexMeshCollideConstraint: public Constraint
		{
		public:
			ConvexMeshCollideConstraint(Particle& p,const ConvexMesh& mesh);
			virtual void Satisfy();
		private:
			Particle& particle;
			const ConvexMesh& convexMesh;
		};

		class MeshCollideConstraint: public Constraint
		{
		public:
			MeshCollideConstraint(Particle& p,const Mesh& mesh);
			virtual void Satisfy();
		private:
			Particle& particle;
			const Mesh& mesh;
		};
	}
}

#endif
