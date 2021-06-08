#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		class CPointCloudVisualizer;

		class CInteractionCallback
		{
		public:
			CInteractionCallback(pcl::visualization::PCLVisualizer* vVisualizer);

			void keyboardCallback(const pcl::visualization::KeyboardEvent& vEvent);

			void mouseCallback(const pcl::visualization::MouseEvent& vEvent);

			void areaPicking(const pcl::visualization::AreaPickingEvent& vEvent);

			~CInteractionCallback() = default;

		private:
			bool m_KeyPressStatus[256] = { false };
			bool m_MousePressStatus[2] = { false };

			bool m_UnwantedMode = true;

			CPointCloudVisualizer* m_pVisualizer;
		};
	}
}
