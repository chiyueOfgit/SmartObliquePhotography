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
			int Id = -1;
		};

		class CInteractionCallback;

		class CPointCloudVisualizer : public hiveDesignPattern::CSingleton<CPointCloudVisualizer>
		{
		public:
			~CPointCloudVisualizer();

			bool init(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt = true);

			void reset(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt = true);

			void refresh(const std::vector<std::size_t>& vPointLabel, bool vResetCamera = false);
			void refresh(std::size_t vTileIndex, const std::vector<std::size_t>& vPointLabel);

			void run();

			void setVisualFlag(int Flag) { _ASSERTE(Flag >= 0); m_VisualFlag = Flag; }

			void setPointRenderSize(double vSize);

			std::vector<RetouchCloud_t::Ptr> getUserCloudSet() { return m_UserCloudSet; }
			void addUserPointCloud(RetouchCloud_t::Ptr vCloud) { m_UserCloudSet.push_back(vCloud); }
			void removeAllUserPointCloud() { m_UserCloudSet.clear(); }
			int addUserColoredPoints(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);
			void removeUserColoredPoints(int vId);
			void removeAllUserColoredPoints() { m_UserColoredPoints.clear(); }
			void addTextureMesh(const pcl::TextureMesh& vMesh) { m_MeshSet.push_back({ "", vMesh}); }
			void removeAllTextureMesh() { m_MeshSet.clear(); }
			/*void jumpToThreeView(EView vViewType);
			void showBoundingBox();*/

		private:
			CPointCloudVisualizer();

			void __autoLod();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			std::vector<VisualCloud_t::Ptr> m_TileSet;
			std::vector<std::size_t> m_OffsetSet;
			std::size_t m_NumPoints;
			std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> m_TileBoxSet;
			
			std::vector<std::pair<std::string, pcl::TextureMesh>> m_MeshSet;

			std::vector<SHighlightPoints> m_UserColoredPoints;
			std::vector<RetouchCloud_t::Ptr> m_UserCloudSet;

			Eigen::Vector2d m_WindowSize;
			const std::string m_CloudName = "Cloud2Show";

			Eigen::Vector3i m_LitterColor = { 255, 0, 0 };
			Eigen::Vector3i m_BackgroundColor = { 0, 0, 255 };

			uint32_t m_VisualFlag = EVisualFlag::ShowCloud | EVisualFlag::ShowUserCloud | EVisualFlag::ShowMesh;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
			friend pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();
			friend int hiveHighlightPointSet(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);
		};
	}
}
