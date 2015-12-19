#ifndef IFORCE_H
#define IFORCE_H
#include "Vector3.h"

namespace ajg
{
	namespace physics
	{
	
		class IForce
		{
		public:
			virtual Vector3 Apply()=0;
			virtual ~IForce()=0;
		};

		inline IForce::~IForce()
		{
		}
		
	}
}

#endif
