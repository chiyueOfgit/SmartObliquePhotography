#include "pch.h"
#include "PCLPointCloudWrapper.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
bool CPCLPointCloudWrapper::init(std::uint8_t vPointTypeFlag)
{
	if (vPointTypeFlag == static_cast<std::uint8_t>(EPointInfo::POSITION))
	{
		return true;
	}

	if (vPointTypeFlag == (static_cast<std::uint8_t>(EPointInfo::POSITION) | static_cast<std::uint8_t>(EPointInfo::NORMAL) | static_cast<std::uint8_t>(EPointInfo::COLOR)))
	{
		return true;
	}

	_HIVE_OUTPUT_WARNING("Fail to build PCL point cloud due to unknown point type flag.");
	return false;
}