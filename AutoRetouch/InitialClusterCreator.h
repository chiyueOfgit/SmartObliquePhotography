#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IInitialClusterCreator : public IPointClassifier
		{
		public:
			IInitialClusterCreator() = default;
			~IInitialClusterCreator() = default;

			virtual void runV(const pcl::Indices& vUserInputSet, EPointLabel vLabel, double vRadius);

		private:
			virtual IPointCluster* __createInitialClusterV(const pcl::Indices& vUserInputSet, double vRadius) = 0;
		};
	}
}