#pragma once

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

			EPointLabel getClusterLabel() const { return m_Label; }

			virtual double computeDistanceV(std::uint64_t vPointIndex) const = 0;

		private:
			EPointLabel m_Label = EPointLabel::UNDETERMINED;
		};
	}
}
