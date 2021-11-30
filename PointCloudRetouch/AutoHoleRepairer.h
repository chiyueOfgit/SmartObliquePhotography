#pragma once
#include "Image.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CAutoHoleRepairer
		{
		public:
			CAutoHoleRepairer() = default;
			~CAutoHoleRepairer() = default;

			void execute(const Eigen::Vector2i& vResolution, std::vector<pcl::index_t>& vPointCloudIndices, std::vector<pcl::PointXYZRGBNormal>& voNewPointSet);
			void setPointDistributionSet(std::vector<std::vector<std::vector<pcl::index_t>>>& vPointDistributionSet);

			void saveTexture(const std::string& vPath, const hiveObliquePhotography::CImage<float>& vTexture, bool vIsReverse);
#ifdef _UNIT_TEST
			void repairImageByMipmap(CImage<float>& vioHoleImage, std::vector<Eigen::Vector2i>& voHoleSet) { __repairImageByMipmap(vioHoleImage, voHoleSet); };
#endif
		
		private:
			void __repairImageByScan(CImage<float>& vioHoleImage, std::vector<Eigen::Vector2i>& voHoleSet);
			void __repairImageByRound2Center(CImage<float>& vioHoleImage, std::vector<Eigen::Vector2i>& voHoleSet);
			void __repairImageByMipmap(CImage<float>& vioHoleImage, std::vector<Eigen::Vector2i>& voHoleSet);
			void __executeMeanFilter(CImage<float>& vioWithoutHoleImage, int vKernel);
			std::vector<pcl::PointXYZRGBNormal> __generateNewPoint(const std::vector<Eigen::Vector2i>& vHoleSet, const CImage<float>& vWithoutHoleMap);
			
			std::vector<std::vector<std::vector<pcl::index_t>>> m_PointDistributionSet;
			
		};
	}
}
