#pragma once
#include <map>
#include <string>

namespace hiveObliquePhotography
{
	class CPointCloudTile;

	class CPointCloudScene
	{
	public:
		CPointCloudScene();
		~CPointCloudScene();

	private:
		std::map<std::string, CPointCloudTile*> m_PointCloudTileMap;
	};
}

