#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CBoundaryDetector : public IPointClassifier
		{
		public:
			CBoundaryDetector() = default;
			~CBoundaryDetector() = default;

			virtual void runV(std::vector<pcl::index_t>& vioBoundarySet, const hiveConfig::CHiveConfig* vConfig);	//vio: 非空传进指定候选点集，空传进为整个场景

		private:

		};
	}
}
