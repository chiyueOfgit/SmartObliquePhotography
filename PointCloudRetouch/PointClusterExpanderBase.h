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

			virtual std::vector<pcl::index_t>& getExpandPoints() { return m_ExpandPoints; };

		private:
			std::vector<pcl::index_t> m_ExpandPoints;
		};
	}
}