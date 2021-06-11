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
			~CBinaryClassifierByVFHAlg() override = default;

			void runV() override;

		private:
			inline std::set<std::uint64_t> __getRemainIndex();
		};
	}
}
