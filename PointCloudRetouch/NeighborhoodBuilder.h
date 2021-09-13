#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CNeighborhood;   //FIXME-014：这个在什么地方用了？
		class CPointLabelSet;

		class INeighborhoodBuilder : public hiveDesignPattern::IProduct
		{
		public:
			INeighborhoodBuilder() = default;
			virtual ~INeighborhoodBuilder();

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, PointCloud_t::Ptr vPointCloudScene, const CPointLabelSet* vPointLabelSet);

			std::vector<pcl::index_t> buildNeighborhood(pcl::index_t vSeed, std::string& vType, float vPara) const;  //FIXME-014：对任何一种建立邻域的方法，如果要传入参数，只需要传入一个参数vPara就可以？
			std::vector<pcl::index_t> buildNeighborhood(pcl::index_t vSeed) const;
			void reset();

		protected:
			PointCloud_t::Ptr m_pPointCloudScene = nullptr;

		private:
			const CPointLabelSet* m_pPointLabelSet = nullptr;  //FIXME-014：建立邻域和这个有什么必然联系？

			virtual void __extraInitV(const hiveConfig::CHiveConfig* vConfig) {}
			virtual std::vector<pcl::index_t> __buildNeighborhoodV(pcl::index_t vSeed, std::string& vType, float vPara) const = 0;  
			virtual std::vector<pcl::index_t> __buildNeighborhoodV(pcl::index_t vSeed) const = 0;
		};
	}
}