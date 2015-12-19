#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Vector3.h"

namespace ajg
{
    namespace physics
	{
        const Scalar PENETRATION_EPSILON = 0.0001f;
        const Scalar COLLISION_EPSILON = 0.1f;
        const Scalar RESTITUTION = 0.8f;
        const Scalar TIME_EPSILON = 0.00001f;
		const Scalar FRICTIONAL_COEFFICIENT = 0.2f;
    }
}

#endif
