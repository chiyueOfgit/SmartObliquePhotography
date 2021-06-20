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
		};
	}
}

