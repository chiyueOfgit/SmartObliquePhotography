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
			~CBinaryClassifierAlg() override = default;

			virtual void runV();

		protected:
			std::vector<IPointCluster*> m_ClusterSet;
			pcl::IndicesPtr __getUnknownIndices();
			SBox __createExecuteArea() const;
		};
	}
}

