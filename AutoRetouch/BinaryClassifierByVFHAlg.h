#pragma once
#include "BinaryClassifierAlg.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster;

		class CBinaryClassifierByVFHAlg : public CBinaryClassifierAlg
		{
		public:
			CBinaryClassifierByVFHAlg() = default;
			~CBinaryClassifierByVFHAlg() = default;

			virtual void runV(const std::vector<IPointCluster*>& vInputClusterSet) override;

		};
	}
}
