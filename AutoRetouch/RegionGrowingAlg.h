#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
		class CRegionGrowingAlg : public IPointClassifier
		{
		public:
			CRegionGrowingAlg() = default;
			~CRegionGrowingAlg() override = default;

			void runV(const pcl::Indices& vSeeds, EPointLabel vDstLabel);

		private:
			template<typename T>
			static bool __testAndUpdateMask(T& vioSubject, const T& vMask);//TODO: move to Common£¿
			
			virtual void __initValidation(const pcl::Indices& vSeeds, PointCloud_t::ConstPtr vCloud) {}
			virtual bool __validatePointV(pcl::index_t vTestIndex, PointCloud_t::ConstPtr vCloud) const { return true; }
		};
	}
}
