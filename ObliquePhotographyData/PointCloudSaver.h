#pragma once

namespace hiveObliquePhotography
{
	class IPointCloudSaver : public hiveDesignPattern::IProduct
	{
	public:
		IPointCloudSaver() = default;
		~IPointCloudSaver() override = default;

		virtual void saveDataToFileV(const PointCloud_t& vPointCloud, const std::string& vFilePath) = 0;
		
	private:
	};
}
