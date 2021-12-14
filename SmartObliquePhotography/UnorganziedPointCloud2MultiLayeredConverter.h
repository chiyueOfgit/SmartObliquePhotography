#pragma once
#include <string>
#include <future>

namespace hiveObliquePhotography
{
	class CMultiLayeredPointCloud;

	class CUnorganziedPointCloud2MultiLayeredConverter
	{
	public:
		CUnorganziedPointCloud2MultiLayeredConverter() = default;
		~CUnorganziedPointCloud2MultiLayeredConverter() = default;

		CMultiLayeredPointCloud* convertUnorganizedPointCloud2MultiLayered();

	private:
		struct SActiveTask
		{
			std::string _FileName;
			std::uint32_t _TileX = 0;
			std::uint32_t _TileY = 0;
			std::future<bool> _Result;
		};

		std::vector<SActiveTask> m_ActiveTaskSet;
		CMultiLayeredPointCloud* m_pMultiLayeredPointCloud = nullptr;

		bool __convertSingleUnorganziedPointCloud(const std::string& vFileName);

		SActiveTask& __updateTaskStatus();

		void __notifyPointCloudConvertIsDone(std::uint32_t vTileX, std::uint32_t vTileY);
		void __waitAllActiveTaskDone();
	};
}
