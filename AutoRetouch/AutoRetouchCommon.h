#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		const std::string CLASSIFIER_BINARY = "Binary";
		const std::string CLASSIFIER_BINARY_VFH = "Binary_vfh";
		const std::string CLASSIFIER_REGION_GROW = "RegionGrow";
		const std::string CLASSIFIER_REGION_GROW_COLOR = "RegionGrow_color";

		enum class EPointLabel : unsigned char
		{
			DISCARDED,
			KEPT,
			UNWANTED,
			UNDETERMINED,
			FILLED,
		};

		struct SPointLabelChange
		{
			std::uint64_t Index = UINT_MAX;
			EPointLabel SrcLabel;
			EPointLabel DstLabel;
			float Confidence = 1.0f;

			friend bool operator < (const SPointLabelChange& vLeft, const SPointLabelChange& vRight) { return vLeft.Index < vRight.Index; }
		};
	}
}