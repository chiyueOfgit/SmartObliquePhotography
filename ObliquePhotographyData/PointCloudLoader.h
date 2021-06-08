#pragma once

namespace hiveObliquePhotography
{
	using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
	
	class IPointCloudLoader : public hiveDesignPattern::IProduct
	{
	public:
		IPointCloudLoader() = default;
		~IPointCloudLoader() override = default;

		PointCloud_t::Ptr loadDataFromFile(const std::string& vFileName);

	private:
		virtual bool __loadDataFromFileV(const std::string& vFileName, PointCloud_t::Ptr voPointCloud) = 0;
	};
}