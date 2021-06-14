#pragma once
#include <cstdint>

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CNeighborhood;

		class INeighborhoodBuilder
		{
		public:
			INeighborhoodBuilder() = default;
			virtual ~INeighborhoodBuilder() = default;

			CNeighborhood* buildNeighborhood(std::uint64_t vSeed);

		private:

		};
	}
}