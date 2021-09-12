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

			void init(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt = true);

			void reset(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt = true);

			void refresh(const std::vector<std::size_t>& vPointLabel, bool vResetCamera = false);

			void setPointRenderSize(double vSize);

			void run();

			void setVisualFlag(int Flag) { _ASSERTE(Flag >= 0); m_VisualFlag = Flag; }

			std::vector<RetouchCloud_t::Ptr> getUserCloudSet() { return m_UserCloudSet; }
			void addUserPointCloud(RetouchCloud_t::Ptr vCloud) { m_UserCloudSet.push_back(vCloud); }
			void removeAllUserPointCloud() { m_UserCloudSet.clear(); }
			int addUserColoredPoints(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);
			int addUserColoredPointsAsNewCloud(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor, const Eigen::Vector3f& vDeltaPos = { 0.0f, 0.0f, 0.0f }, double vPointSize = 3.0);
			void removeUserColoredPoints(int vId);
			void removeAllUserColoredPoints() { m_UserColoredPoints.clear(); }
			void addTextureMesh(const pcl::TextureMesh& vMesh) { m_MeshSet.push_back({ "", vMesh}); }
			void removeAllTextureMesh() { m_MeshSet.clear(); }
			/*void jumpToThreeView(EView vViewType);
			void showBoundingBox();*/

			std::vector<Eigen::Vector3d> getTileCenter()const { return m_TileCenter; }
			std::string getName()const { return m_CloudName; }
		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			std::vector<VisualCloud_t::Ptr> m_TileSet;
			std::vector<std::size_t> m_OffsetSet;
			std::size_t m_NumPoints;
			//std::pair<Eigen::Vector3f, Eigen::Vector3f> m_AABB;
			std::vector<Eigen::Vector3d> m_TileCenter;
			
			std::vector<std::pair<std::string, pcl::TextureMesh>> m_MeshSet;

			std::vector<RetouchCloud_t::Ptr> m_UserCloudSet;

			Eigen::Vector2d m_WindowSize;

			std::vector<SHighlightPoints> m_UserColoredPoints;

			std::vector<Eigen::Vector3i> m_MainColors;

			const std::string m_CloudName = "Cloud2Show";

			std::tuple<int, int, int> m_LitterColor = { 255, 0, 0 };
			std::tuple<int, int, int> m_BackgroundColor = { 0, 0, 255 };

			uint32_t m_VisualFlag = EVisualFlag::ShowCloud | EVisualFlag::ShowUserCloud | EVisualFlag::ShowMesh;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
			friend pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();
			friend int hiveHighlightPointSet(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);
			friend void hiveRemoveAllShapes();

		};
	}
}
