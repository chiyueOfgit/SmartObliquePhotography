#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		struct SPointsRecord
		{
			std::vector<pcl::index_t> PointSet;
			Eigen::Vector3i Color{0, 0, 0};
			int Lifetime = 1;
		};

		class CInteractionCallback;

		class CPointCloudVisualizer : public hiveDesignPattern::CSingleton<CPointCloudVisualizer>
		{
		public:
			~CPointCloudVisualizer();

			void init(PointCloud_t::Ptr vPointCloud, bool vIsInQt = true);

			void reset(PointCloud_t::Ptr vPointCloud = nullptr, bool vIsInQt = true);

			void refresh(const std::vector<std::size_t>& vPointLabel, bool vResetCamera = false);

			void run();

		private:
			CPointCloudVisualizer();

			void __setPointsAndColor(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor, bool vIsTemp = true);

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			PointCloud_t::Ptr m_pSceneCloud = nullptr;

			std::vector<SPointsRecord> m_UserColoredPoints;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
			friend pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();
			friend void hiveSetPointsColor(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor, bool vIsTemp);
		};
	}
}
