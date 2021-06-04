#pragma once

namespace hiveObliquePhotography
{
	namespace visualizer
	{
		class CPointCloudVisualizer : protected pcl::visualization::PCLVisualizer, public hiveDesignPattern::CSingleton<CPointCloudVisualizer>
		{
		public:
			~CPointCloudVisualizer();

			void refresh(bool vResetCamera = false);

		private:
			CPointCloudVisualizer();

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
		};
	}
}
