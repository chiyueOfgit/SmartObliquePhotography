#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		namespace KEYWORD
		{
			const std::string VFH_FEATURE = "VFH_FEATURE";
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