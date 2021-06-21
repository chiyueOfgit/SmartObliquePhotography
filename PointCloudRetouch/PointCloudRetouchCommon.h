#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		namespace KEYWORD
		{

		}
		enum class EPointLabel : unsigned char
		{
			DISCARDED,
			KEPT,
			UNWANTED,
			UNDETERMINED,
			FILLED,
		};

		template <typename T>
		T NormalDistribution(T vX, T vDelta  = 1)
		{
			return exp(-(vX * vX) / (2 * vDelta * vDelta)) / (2.50662827464 * vDelta);
		}
	}
}