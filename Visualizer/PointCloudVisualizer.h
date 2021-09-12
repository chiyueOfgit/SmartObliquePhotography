#pragma once
#include "InteractionCallback.h"
#include "VisualizationConfig.h"

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

			template<class TPointCloud>
			void init(TPointCloud::Ptr vPointCloud, bool vIsInQt = true)
			{
				_ASSERTE(vPointCloud);
				m_UserColoredPoints.clear();
				m_UserCloudSet.clear();

				m_pSceneCloud.reset(new VisualCloud_t);
				pcl::copyPointCloud(*vPointCloud, *m_pSceneCloud);

				std::vector<Eigen::Vector3f> PositionSet;
				for (auto Point : *vPointCloud)
					PositionSet.push_back(Point.getVector3fMap());
				m_AabbBox = calcAABB(PositionSet);

				m_pPCLVisualizer = new pcl::visualization::PCLVisualizer("Visualizer", !vIsInQt);
				m_pCallback = new CInteractionCallback(m_pPCLVisualizer);
				m_pPCLVisualizer->setBackgroundColor(0.2, 0.2, 0.2);
				m_pPCLVisualizer->setShowFPS(false);

				auto OptionLitterColor = CVisualizationConfig::getInstance()->getAttribute<std::tuple<int, int, int>>(LITTER_HIGHLIGHT_COLOR);
				auto OptionBackgroundColor = CVisualizationConfig::getInstance()->getAttribute<std::tuple<int, int, int>>(BACKGROUND_HIGHLIGHT_COLOR);
				if (OptionLitterColor.has_value() && OptionBackgroundColor.has_value())
				{
					m_LitterColor = OptionLitterColor.value();
					m_BackgroundColor = OptionBackgroundColor.value();
				}
			}

			template<class TPointCloud>
			void reset(TPointCloud::Ptr vPointCloud = nullptr, bool vIsInQt = true)
			{
				m_pPCLVisualizer->removeAllPointClouds();
				delete m_pPCLVisualizer;
				delete m_pCallback;
				m_UserColoredPoints.clear();
				m_UserCloudSet.clear();
				init<TPointCloud>(vPointCloud, vIsInQt);
				if (vPointCloud != nullptr)
				{
					m_pSceneCloud.reset(new VisualCloud_t);
					pcl::copyPointCloud(*vPointCloud, *m_pSceneCloud);
				}
			}

			void refresh(const std::vector<std::size_t>& vPointLabel, bool vResetCamera = false);

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
			void jumpToThreeView(EView vViewType);
			void showBoundingBox();
		
		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			VisualCloud_t::Ptr m_pSceneCloud = nullptr;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> m_AabbBox;

			std::vector<std::pair<std::string, pcl::TextureMesh>> m_MeshSet;

			std::vector<RetouchCloud_t::Ptr> m_UserCloudSet;

			Eigen::Vector2d m_WindowSize;

			std::vector<SHighlightPoints> m_UserColoredPoints;

			std::vector<Eigen::Vector3i> m_MainColors;

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
