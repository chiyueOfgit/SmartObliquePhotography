#pragma once

namespace hiveObliquePhotography
{
	class CPointCloudScene : public hiveDesignPattern::CSingleton<CPointCloudScene>
	{
	public:
		~CPointCloudScene() override;

		PointCloud_t::Ptr loadScene(const std::vector<std::string>& vFileNameSet);
		bool saveScene(PointCloud_t& vPointCloud, std::string vFileName);
		
		std::size_t getNumTiles() const { return m_PointCloudTileMap.size(); }
		
		auto getTileSet() const { return m_TileSet; }

		void clear();

	private:
		CPointCloudScene() = default;

		std::vector<PointCloud_t::Ptr> m_TileSet;
		std::map<std::string, PointCloud_t::Ptr> m_PointCloudTileMap;
		PointCloud_t::Ptr m_pPointCloudScene;

	friend class hiveDesignPattern::CSingleton<CPointCloudScene>;
	};
}

