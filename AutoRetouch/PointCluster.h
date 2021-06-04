#pragma once
#include "AutoRetouchCommon.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster
		{
		public:
			IPointCluster() = default;
			IPointCluster(EPointLabel vLabel) : m_Label(vLabel) {}
			virtual ~IPointCluster() = default;

			virtual double computeDistanceV(std::uint64_t vPointIndex) const;

		private:
			EPointLabel m_Label = EPointLabel::UNDETERMINED;
		};
	}
}
