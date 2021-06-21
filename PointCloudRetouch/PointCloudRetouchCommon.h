#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;

		namespace KEYWORD
		{
			const std::string VFH_FEATURE = "VFH_FEATURE";
			const std::string COLOR_FEATURE = "COLOR_FEATURE";
		}
		enum class EPointLabel : unsigned char
		{
			DISCARDED,
			KEPT,
			UNWANTED,
			UNDETERMINED,
			FILLED,
		};
	}
}