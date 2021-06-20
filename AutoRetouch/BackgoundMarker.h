#pragma once
#include "Task.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CBackgoundMarker : virtual public ITask, virtual public hiveDesignPattern::CSingleton<CBackgoundMarker>
		{
		public:
			~CBackgoundMarker();

		private:
			CBackgoundMarker() = default;

		friend class hiveDesignPattern::CSingleton<CBackgoundMarker>;
		};
}
}
