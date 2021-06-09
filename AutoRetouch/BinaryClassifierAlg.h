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
			virtual ~CBinaryClassifierAlg() = default;

			virtual void runV();

		protected:
			std::vector<IPointCluster*> m_ClusterSet;

			SBox m_AABB;
		};
	}
}

