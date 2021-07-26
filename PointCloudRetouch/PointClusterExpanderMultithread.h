#pragma once
#include "PointClusterExpanderBase.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;

		class CPointClusterExpanderMultithread : public IPointClusterExpanderBase
		{
		public:
			CPointClusterExpanderMultithread() = default;
			~CPointClusterExpanderMultithread() = default;

			virtual void runV(const CPointCluster* vCluster);

			//virtual const std::vector<pcl::index_t>& getExpandPoints() override { return m_ExpandPoints; }

#ifdef _UNIT_TEST
			std::vector<pcl::index_t> initExpandingCandidateQueue(const CPointCluster* vCluster) { return __initExpandingCandidateQueue(vCluster); }
			//virtual double getRunTime() override { return m_RunTime; };
#endif // _UNIT_TEST

		private:
			std::vector<pcl::index_t> __initExpandingCandidateQueue(const CPointCluster* vCluster);

			bool __isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp);

			//std::vector<pcl::index_t> m_ExpandPoints;

			//double m_RunTime = 0;
		};
	}
}

