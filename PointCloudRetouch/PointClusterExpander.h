#pragma once
#include "PointClusterExpanderBase.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;

		class CPointClusterExpander : public IPointClusterExpanderBase
		{
		public:
			CPointClusterExpander() = default;
			~CPointClusterExpander() = default;

			virtual void runV(const CPointCluster* vCluster);

			virtual const std::vector<pcl::index_t>& getExpandPoints() const { return m_ExpandPoints; }

#ifdef _UNIT_TEST
			std::queue<pcl::index_t> initExpandingCandidateQueue(const CPointCluster* vCluster) { return __initExpandingCandidateQueue(vCluster); }
#endif // _UNIT_TEST

		private:
			std::queue<pcl::index_t> __initExpandingCandidateQueue(const CPointCluster* vCluster);

			bool __isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp);

			std::vector<pcl::index_t> m_ExpandPoints;
		};
	}
}

