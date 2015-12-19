#ifndef IRENDERABLE_H
#define IRENDERABLE_H

namespace ajg
{
	namespace physics
	{
		class IRenderable
		{
		public:
			virtual void Render()=0;
			virtual ~IRenderable()=0;
		};
		inline IRenderable::~IRenderable()
		{
		}
	}
}

#endif
