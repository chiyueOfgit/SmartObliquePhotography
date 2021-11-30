#pragma once
#include "PointClusterExpanderBase.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;

		class CPointClusterExpanderMultithread4Auto : public IPointClusterExpander
		{
		public:
			CPointClusterExpanderMultithread4Auto() = default;
			~CPointClusterExpanderMultithread4Auto() = default;

			virtual void runV(const CPointCluster* vCluster) override;
			void setInitialCandidate(const pcl::Indices& vIndices) { m_InitialCandidate = vIndices; }

#ifdef _UNIT_TEST
			std::vector<pcl::index_t> initExpandingCandidateQueue(const CPointCluster* vCluster) { return __initExpandingCandidateQueue(vCluster); }
#endif // _UNIT_TEST

		private:
			std::vector<pcl::index_t> __initExpandingCandidateQueue(const CPointCluster* vCluster);
			bool __isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp);
			
			pcl::Indices m_InitialCandidate;
		};
	}
}

