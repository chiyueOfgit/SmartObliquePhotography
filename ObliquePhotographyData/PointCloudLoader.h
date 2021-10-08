#pragma once

namespace hiveObliquePhotography
{
	class IPointCloudLoader : public hiveDesignPattern::IProduct
	{
	public:
		IPointCloudLoader() = default;
		~IPointCloudLoader() override = default;

		PointCloud_t::Ptr loadDataFromFile(const std::string& vFileName);
		
	private:
		virtual int __loadDataFromFileV(const std::string& vFileName, PointCloud_t::Ptr voPointCloud) = 0;
	};
}