#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		struct SHighlightPoints
		{
			std::vector<pcl::index_t> PointSet;
			Eigen::Vector3i Color{ 0, 0, 0 };
			Eigen::Vector3f DeltaPos{ 0.0f, 0.0f, 0.0f };
			double PointSize = 3.0;
			bool IsNewCloud = false;
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
			int addUserColoredPointsAsNewCloud(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor, const Eigen::Vector3f& vDeltaPos = { 0.0f, 0.0f, 0.0f }, double vPointSize = 3.0);
			void removeUserColoredPoints(int vId);
			void removeAllUserColoredPoints() { m_UserColoredPoints.clear(); }

		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			PointCloud_t::Ptr m_pSceneCloud = nullptr;

			Eigen::Vector2d m_WindowSize;

			std::vector<SHighlightPoints> m_UserColoredPoints;

			std::vector<Eigen::Vector3i> m_MainColors;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
			friend pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();
			friend int hiveHighlightPointSet(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);
			friend void hiveRemoveAllShapes();

		};
	}
}
