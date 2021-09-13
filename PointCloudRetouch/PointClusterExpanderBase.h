#pragma once
#include "pch.h"
#include "PointClassifier.h"
#include "PointCluster.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class IPointClusterExpander : public IPointClassifier
		{
		public:
			IPointClusterExpander() = default;
			~IPointClusterExpander() = default;

			virtual void runV(const CPointCluster* vCluster) { _ASSERTE(vCluster); };

			virtual const std::vector<pcl::index_t>& getExpandedPointSet() { return m_ExpandedPointSet; };  //FIXME-014: 这个函数做成虚函数是什么意思？

#ifdef _UNIT_TEST
			virtual double getRunTime() { return m_RunTime; };
#endif // _UNIT_TEST

		protected:
			std::vector<pcl::index_t> m_ExpandedPointSet;
#ifdef _UNIT_TEST
			double m_RunTime = 0;
#endif // _UNIT_TEST
		};
	}
}