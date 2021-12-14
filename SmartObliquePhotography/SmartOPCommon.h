#pragma once
#include <stdint.h>
#include <string>
#include <Eigen/Core>

namespace hiveObliquePhotography
{
	enum class EPointCategory : std::uint16_t
	{
		UNDEFINED = 1,
		GROUND = 2,
		ROAD = 4,
		BUILDING = 8,
		BUSH = 16,
		RIVER = 32,
	};

	enum class EPointInfo : std::uint8_t
	{
		POSITION = 1,
		COLOR = 2,
		NORMAL = 4,
	};

	struct SPointColor
	{
		std::uint8_t _r;
		std::uint8_t _g;
		std::uint8_t _b;
	};

	struct SUnorganizedPointCloud  //用于保存从一个文件中读入的点云数据
	{
		std::vector<Eigen::Vector3f> _PositionSet;
		std::vector<SPointColor>     _ColorSet;

		void clear()
		{
			_PositionSet.clear();
			_ColorSet.clear();
		}

		bool isValid() const
		{
			if (_PositionSet.empty()) return false;
			return (_PositionSet.size() == _ColorSet.size());
		}
	};

	const std::uint32_t INVALID_BLOCK_COORD = UINT_MAX;

	namespace Keywords
	{
		const std::string SCENE_LONGITUDE_SPAN = "Scene_Longitude_Span";
		const std::string SCENE_LATITUDE_SPAN = "Scene_Latitude_Span";
		const std::string SCENE_WIDTH_IN_CELL = "Scene_Width_In_Cell";
		const std::string SCENE_HEIGHT_IN_CELL = "Scene_Height_In_Cell";
		const std::string CELL_SCALE = "Cell_Scale";
		const std::string EXPECTED_BLOCK_SIZE_IN_MB = "Expected_Block_Size_In_MB";
		
		const std::string MAX_NUM_POINT_CLOUD_CONVERT_TASK = "Max_Num_Point_Cloud_Convert_Task";
		const std::string MAX_NUM_LOOP_WAIT_POINT_CLOUD_CONVERT_DONE = "Max_Num_Loop_Wait_Point_Cloud_Convert_Done";

		const std::string POINT_CLOUD_FILE_LOADER_SIG = "";
	}
}