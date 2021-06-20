#pragma once

namespace hiveObliquePhotography
{
	class IPointCloudSaver : public hiveDesignPattern::IProduct
	{
	public:
		IPointCloudSaver() = default;
		~IPointCloudSaver() override = default;

		virtual void saveDataToFile(const PointCloud_t& vPointCloud, const std::string& vFilePath) = 0;

	private:
	};
}
