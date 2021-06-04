#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster;

		class CBinaryClassifierAlg : public IPointClassifier
		{
		public:
			CBinaryClassifierAlg() = default;
			~CBinaryClassifierAlg() = default;

			virtual void runV(std::vector<IPointCluster*> vInputClusterSet);

		private:
			std::vector<IPointCluster*> m_ClusterSet;
		};
	}
}

