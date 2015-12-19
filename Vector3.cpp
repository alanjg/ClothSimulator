#include "Vector3.h"
#include <ostream>

namespace ajg {
	namespace physics {

		
		std::ostream& operator<<(std::ostream& out,const Vector3& v)
		{
			return out<<"("<<v[0]<<","<<v[1]<<","<<v[2]<<")";
		}
		
	
	}
}
