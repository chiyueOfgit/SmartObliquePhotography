#pragma once
#include <string>

namespace hiveObliquePhotography
{
	class CPointCloudTile;

	class IPointCloudTileLoader
	{
	public:
		IPointCloudTileLoader();
		~IPointCloudTileLoader();

		[nodiscard] bool loadData(const std::string& vFileName, CPointCloudTile *voPointCloudTile);

	private:
		virtual bool __loadDataV(const std::string& vFileName, CPointCloudTile* voPointCloudTile);
	};
}