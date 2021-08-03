#include "pch.h"
#include "PointCloudRetouchConfig.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchConfig::__defineAttributesV()
{
	_defineAttribute("NEIGHBOR_BUILDER",	hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("TASK",				hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("CLUSTER",				hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("FEATURE",				hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("CLASSIFIER",			hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("OUTLIER",				hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("HOLE_REPAIRER",		hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("BOUNDARY_DETECTOR",	hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("TEXTURE_SYNTHESIZER", hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("POINT_CLOUD_RETOUCN_CONFIG", hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
	_defineAttribute("UP", hiveConfig::EConfigDataType::ATTRIBUTE_VEC3F);
	_defineAttribute("SIG", hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
	_defineAttribute("SEARCH_MODE", hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
	_defineAttribute("NEAREST_N", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("RADIUS", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("INIT_RESOLUTION", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("HARDNESS_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("DEPTH_OFFSET", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("EXPECT_PROBABILITY", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("DISTANCE_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("DISTANCE_TOLERANCE", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("CONVOLUTION_KERNEL_SIZE", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("COLOR_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("NUM_MAIN_COLORS", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("SEARCH_RADIUS", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("MIN_NEIGHBORS_IN_RADIUS", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("POINT_FILTER_CONDITION", hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
	_defineAttribute("SMALL_SCALE_RADIUS", hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
	_defineAttribute("LARGE_SCALE_RADIUS", hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
	_defineAttribute("SEARCH_MODE_BOUNDARY_DETECTOR", hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
	_defineAttribute("NEAREST_N_BOUNDARY_DETECTOR", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("BOUNDARY_DISTANCE_TOLERANCE", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
	_defineAttribute("BOUNDARY_SIZE_TOLERANCE", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	_defineAttribute("BOUNDARY_IF_CLOSED_TOLERANCE", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
}