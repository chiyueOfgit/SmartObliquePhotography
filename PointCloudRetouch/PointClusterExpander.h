#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;

		class CPointClusterExpander : public IPointClassifier
		{
		public:
			CPointClusterExpander() = default;
			~CPointClusterExpander() = default;

			virtual void runV(const CPointCluster* vCluster);
#ifdef _UNIT_TEST
			void initExpandingCandidateQueue(const CPointCluster* vCluster, std::queue<pcl::index_t>& voCandidateQueue) { __initExpandingCandidateQueue(vCluster, voCandidateQueue); }
#endif // _UNIT_TEST
		private:
			std::queue<pcl::index_t> __initExpandingCandidateQueue(const CPointCluster* vCluster);

			bool __isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp);
		};
	}
}

