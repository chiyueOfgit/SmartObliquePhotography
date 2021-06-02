#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
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
			std::uint64_t Index;
			EPointLabel SrcLabel;
			EPointLabel DstLabel;
		};
	}
}