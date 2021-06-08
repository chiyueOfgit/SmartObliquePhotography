#pragma once

namespace hiveObliquePhotography
{
	using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
	
	class CPointCloudScene : public hiveDesignPattern::CSingleton<CPointCloudScene>
	{
	public:
		~CPointCloudScene() override;

		PointCloud_t::Ptr loadScene(const std::vector<std::string>& vFileNameSet);

		std::size_t getNumTiles() const { return m_PointCloudTileMap.size(); }

		void clear();

	private:
		CPointCloudScene() = default;

		std::map<std::string, PointCloud_t::Ptr> m_PointCloudTileMap;
		PointCloud_t::Ptr m_pPointCloudScene;

	friend class hiveDesignPattern::CSingleton<CPointCloudScene>;
	};
}

