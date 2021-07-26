#pragma once
#include "pch.h"
#include "PointClassifier.h"
#include "PointCluster.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class IPointClusterExpanderBase : public IPointClassifier
		{
		public:
			IPointClusterExpanderBase() = default;
			~IPointClusterExpanderBase() = default;

			virtual void runV(const CPointCluster* vCluster) { _ASSERTE(vCluster); };

			virtual const std::vector<pcl::index_t>& getExpandPoints() { return m_ExpandPoints; };

#ifdef _UNIT_TEST
			virtual double getRunTime() { return m_RunTime; };
#endif // _UNIT_TEST

		protected:
			std::vector<pcl::index_t> m_ExpandPoints;
			double m_RunTime = 0;
		};
	}
}