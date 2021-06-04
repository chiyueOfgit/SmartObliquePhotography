#pragma once

namespace hiveObliquePhotography
{
	class CPointCloudTile;

	class CPointCloudScene : public hiveDesignPattern::CSingleton<CPointCloudScene>
	{
	public:
		~CPointCloudScene();

		pcl::PointCloud<pcl::PointSurfel>* loadScene(const std::vector<std::string>& vFileNameSet);

		std::size_t getNumTiles() const { return m_PointCloudTileMap.size(); }

		void clear();

	private:
		CPointCloudScene() = default;

		std::map<std::string, CPointCloudTile*> m_PointCloudTileMap;
		pcl::PointCloud<pcl::PointSurfel> m_PointCloudScene;

	friend class hiveDesignPattern::CSingleton<CPointCloudScene>;
	};
}

