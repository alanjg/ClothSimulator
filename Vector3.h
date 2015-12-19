#ifndef VECTOR3_H
#define VECTOR3_H
#include <cmath>
#include <iosfwd>

namespace ajg
{
	namespace physics
	{
		typedef float Scalar;

		struct Vector2
		{
			Scalar x,y;
		};

		class Vector3
		{
		public:
			Vector3(Scalar array[3])
			{
				v_[0] = array[0]; 
				v_[1] = array[1];
				v_[2] = array[2];
			}
			Vector3()
			{
			}
			Vector3(Scalar x,Scalar y,Scalar z)
			{
				v_[0] = x;
				v_[1] = y;
				v_[2] = z;
			}
			void Set(Scalar x,Scalar y,Scalar z)
			{
				v_[0] = x;
				v_[1] = y;
				v_[2] = z;
			}
			const Scalar& operator[](int index) const
			{
				return v_[index];
			}
			Scalar& operator[](int index)
			{
				return v_[index];
			}
			Vector3& operator+=(const Vector3& rhs)
			{
				v_[0] += rhs.v_[0];
				v_[1] += rhs.v_[1];
				v_[2] += rhs.v_[2];
				return *this;
			}
			Vector3 operator+(const Vector3& rhs) const
			{
				return Vector3(*this) += rhs;
			}
			Vector3& operator-=(const Vector3& rhs)
			{
				v_[0] -= rhs.v_[0];
				v_[1] -= rhs.v_[1];
				v_[2] -= rhs.v_[2];
				return *this;
			}
			Vector3 operator-(const Vector3& rhs) const
			{
				return Vector3(*this) -= rhs;
			}
			Vector3& operator*=(Scalar rhs)
			{
				v_[0] *= rhs;
				v_[1] *= rhs;
				v_[2] *= rhs;
				return *this;
			}
			friend Vector3 operator*(const Vector3& lhs, Scalar rhs);
			friend Vector3 operator*(Scalar lhs, const Vector3& rhs);
			Vector3& operator/=(Scalar rhs)
			{
				v_[0] /= rhs;
				v_[1] /= rhs;
				v_[2] /= rhs;
				return *this;
			}
			Vector3 operator/(Scalar rhs) const
			{		
				return Vector3(*this) /= rhs;
			}
			Vector3 operator-() const
			{
				return Vector3(*this) *= -1;
			}
			Scalar Dot(const Vector3& rhs) const
			{
				return v_[0]*rhs.v_[0] + v_[1]*rhs.v_[1] + v_[2]*rhs.v_[2];
			}
			Vector3 Cross(const Vector3& rhs) const
			{
				return Vector3(v_[1]*rhs.v_[2]-v_[2]*rhs.v_[1],
								v_[2]*rhs.v_[0]-v_[0]*rhs.v_[2],
								v_[0]*rhs.v_[1]-v_[1]*rhs.v_[0]);
			}
			Scalar Magnitude() const 
			{
				return std::sqrt(MagnitudeSquared());
			}
			Scalar MagnitudeSquared() const
			{
				return Dot(*this);
			}
            Scalar Normalize()
            {
                Scalar mag = Magnitude();
				if(mag==0)
					return mag;

                v_[0] /= mag;
                v_[1] /= mag;
                v_[2] /= mag;
				return mag;
            }
            const Scalar* pointer() const
            {
                return &v_[0]; 
            }
			static Vector3 Zero()
			{
				return Vector3(0,0,0);
			}
			private:
				Scalar v_[3];
		};

		std::ostream& operator<<(std::ostream& out,const Vector3& v);

		inline Vector3 operator*(const Vector3& lhs, Scalar rhs)
		{		
			return Vector3(lhs) *= rhs;
		}
		inline Vector3 operator*(Scalar lhs, const Vector3& rhs)
		{
			return Vector3(rhs) *= lhs;
		}
	}
}
#endif
