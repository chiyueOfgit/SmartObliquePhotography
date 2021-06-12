#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CSpatialClusteringAlg : public IPointClassifier
		{
		public:
			CSpatialClusteringAlg() = default;
			~CSpatialClusteringAlg() = default;

			virtual void runV(const pcl::Indices& vInputSet, EPointLabel vFinalLabel);  //TODO£ºÈçºÎ³éÏó£¿

		private:
		};
	}
}