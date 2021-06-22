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
			std::queue<pcl::index_t> initExpandingCandidateQueue(const CPointCluster* vCluster) { return __initExpandingCandidateQueue(vCluster); }
#endif // _UNIT_TEST
		private:
			std::queue<pcl::index_t> __initExpandingCandidateQueue(const CPointCluster* vCluster);

			bool __isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp);
		};
	}
}

