#pragma once
#include "AutoRetouchCommon.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointCloudAutoRetouchScene : public hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>
		{
		public:
			~CPointCloudAutoRetouchScene();

			void changePointLabel(const std::vector<SPointLabelChange>& vPointLabelChangeRecord);
			void resetPointLabel(const std::vector<SPointLabelChange>& vPointLabelChangeRecord);

		private:
			CPointCloudAutoRetouchScene();

			pcl::PointCloud<pcl::PointSurfel>* m_pPointCloudScenen = nullptr;
			std::vector<EPointLabel> m_PointLabelSet;
			pcl::search::KdTree<pcl::PointSurfel>* m_pGlobalKdTree = nullptr;

		friend class hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>;
		};
	}
}


