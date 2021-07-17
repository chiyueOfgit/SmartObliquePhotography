#pragma once

namespace hiveObliquePhotography
{
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
			PointCloud_t::Ptr m_pFeatureCloud = nullptr;

			friend class hiveDesignPattern::CSingleton<CNormalComplexityVisualization>;


		};
	}
}