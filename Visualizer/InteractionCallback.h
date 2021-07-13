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
			bool m_KeyPressStatus[256] = { false };
			bool m_MousePressStatus[2] = { false };

			double m_Radius = 40.0;
			double m_Hardness = 0.8;

			bool m_UnwantedMode = true;
			bool m_IsRefreshImmediately = true;	//点选是否立即刷新

			CPointCloudVisualizer* m_pVisualizer = nullptr;

			CVisualizationConfig* m_pVisualizationConfig = nullptr;
		};
	}
}
