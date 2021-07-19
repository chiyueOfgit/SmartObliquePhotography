#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		class CPointCloudVisualizer;
	}

	namespace Feature
	{
		struct LAB
		{
			float l;
			float a;
			float b;
		};

		class CColorVisualization : public hiveDesignPattern::CSingleton<CColorVisualization>
		{
		public:
			~CColorVisualization();

			void init(PointCloud_t::Ptr vPointCloud);

			void run(const std::vector<pcl::index_t>& vPoints);

			const auto& getMainBaseColors() const { return m_MainBaseColors; }

		private:
			CColorVisualization();

			std::vector<Eigen::Vector3i> __adjustKMeansCluster(const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vK) const;

			float __calcColorDifferences(const Eigen::Vector3i& vLColor, const Eigen::Vector3i& vRColor) const;
			float __calculateCIEDE2000(const LAB& lab1, const LAB& lab2) const;
			LAB __RGB2LAB(const Eigen::Vector3f& vRGBColor) const;

			std::vector<Eigen::Vector3i> m_MainBaseColors;

			float m_ColorThreshold = 10.0f;
			std::size_t m_MaxNumMainColors = 5;
			float m_MinReduceRatio = 0.8f;

			PointCloud_t::Ptr m_pCloud = nullptr;
			Visualization::CPointCloudVisualizer* m_pVisualizer = nullptr;

			friend class hiveDesignPattern::CSingleton<CColorVisualization>;


		};
	}
}
