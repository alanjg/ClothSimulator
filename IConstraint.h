#ifndef CONSTRAINT_H
#define CONSTRAINT_H

namespace ajg
{
	namespace physics
	{

		class Particle;

		class Constraint
		{
		public:
			virtual void Satisfy()=0;
			virtual ~Constraint()=0;
		};

		inline Constraint::~Constraint()
		{
		}
	}
}

#endif
