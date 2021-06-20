#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IOpResult
		{
		public:
			IOpResult() = default;
			virtual ~IOpResult() = default;

			virtual bool undoV() = 0;
		};
	}
}