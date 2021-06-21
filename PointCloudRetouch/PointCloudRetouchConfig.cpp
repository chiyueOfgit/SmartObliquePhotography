#include "pch.h"
#include "PointCloudRetouchConfig.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchConfig::__defineAttributesV()
{
	_defineAttribute("NEIGHBOR_BUILDER", hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("TASK",			 hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("CLUSTER",		     hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("FEATURE",			 hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("CLASSIFIER",       hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("POINT_CLOUD_RETOUCN_CONFIG", hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);

	_defineAttribute("UP", hiveConfig::EConfigDataType::ATTRIBUTE_VEC3F);
	_defineAttribute("SIG", hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
	_defineAttribute("SEARCH_MODE", hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
	_defineAttribute("NEAREST_N", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("RADIUS", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("INIT_RESOLUTION", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("DEPTH_OFFSET", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("EXPECT_PROBABILITY", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("DISTANCE_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("CONVOLUTION_KERNEL_SIZE", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("COLOR_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("NUM_MAIN_COLORS", hiveConfig::EConfigDataType::ATTRIBUTE_INT);

}