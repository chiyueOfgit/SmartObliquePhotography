#pragma once

namespace hiveObliquePhotography
{
	class CPointCloudScene : public hiveDesignPattern::CSingleton<CPointCloudScene>
	{
	public:
		~CPointCloudScene() override;

		std::vector<PointCloud_t::Ptr> loadScene(const std::vector<std::string>& vFileNameSet);
		bool saveScene(PointCloud_t::Ptr vPointCloud, std::string vFileName);

	private:
		CPointCloudScene() = default;

	friend class hiveDesignPattern::CSingleton<CPointCloudScene>;
	};
}

