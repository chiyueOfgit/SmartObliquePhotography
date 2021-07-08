#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		struct SHighlightPoints
		{
			std::vector<pcl::index_t> PointSet;
			Eigen::Vector3i Color{0, 0, 0};
			int Id = -1;
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

			int addUserColoredPoints(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);
			void removeUserColoredPoints(int vId);
			void removeAllUserColoredPoints() { m_UserColoredPoints.clear(); }

		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			PointCloud_t::Ptr m_pSceneCloud = nullptr;

			std::vector<SHighlightPoints> m_UserColoredPoints;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
			friend pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();
			friend int hiveHighlightPointSet(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);

		};
	}
}
