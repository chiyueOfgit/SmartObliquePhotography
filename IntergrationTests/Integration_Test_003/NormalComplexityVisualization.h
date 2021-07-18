#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		class CPointCloudVisualizer;
	}

	namespace Feature
	{
		class CNormalComplexityVisualization : public hiveDesignPattern::CSingleton<CNormalComplexityVisualization>
		{
		public:
			~CNormalComplexityVisualization();

			void init(PointCloud_t::Ptr vPointCloud);

			void run();

		private:
			CNormalComplexityVisualization();

			PointCloud_t::Ptr m_pCloud = nullptr;
			Visualization::CPointCloudVisualizer* m_pVisualizer = nullptr;

			friend class hiveDesignPattern::CSingleton<CNormalComplexityVisualization>;


		};
	}
}