#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CAutoRetouchConfig;
	}

	namespace Visualization
	{
		class CPointCloudVisualizer;
		class CVisualizationConfig;

		class CInteractionCallback
		{
		public:
			CInteractionCallback(pcl::visualization::PCLVisualizer* vVisualizer);

			void keyboardCallback(const pcl::visualization::KeyboardEvent& vEvent);

			void mouseCallback(const pcl::visualization::MouseEvent& vEvent);

			void pointPicking(const pcl::visualization::PointPickingEvent& vEvent);

			void areaPicking(const pcl::visualization::AreaPickingEvent& vEvent);

			~CInteractionCallback() = default;

		private:
			void __drawHintCircle();

			void __saveIndices(const std::string& vPath, const std::vector<int>& vIndices) const ;
			void __loadIndices(const std::string& vPath, std::vector<int>& voIndices) const;

			pcl::visualization::Camera m_Camera;
			bool m_KeyPressStatus[256] = { false };
			bool m_MousePressStatus[3] = { false };

			int m_PosX = 0;
			int m_PosY = 0;

			double m_Radius = 25.0;
			double m_Hardness = 0.8;

			bool m_UnwantedMode = true;
			bool m_IsRefreshImmediately = true;	//点选是否立即刷新

			CPointCloudVisualizer* m_pVisualizer = nullptr;
			CVisualizationConfig* m_pVisualizationConfig = nullptr;
		};
	}
}
