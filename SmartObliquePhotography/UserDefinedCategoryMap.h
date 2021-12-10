#pragma once
#include "SmartOPCommon.h"

namespace hiveObliquePhotography
{
	class CUserDefinedCategoryMap
	{
	public:
		CUserDefinedCategoryMap();
		~CUserDefinedCategoryMap();

		bool load();

		EPointCategory getCategory(float vLongitude, float vLatitude);

	private:
		float m_MinLongitude = 0;
		float m_MinLatitude = 0;
		float m_MaxLongitude = 0;
		float m_MaxLatitude = 0;
		std::uint32_t m_WidthInPixel = 0;
		std::uint32_t m_HeightInPixel = 0;
		float m_LongitudeSpanPerPixel = 0;
		float m_LatitudeSpanPerPixel = 0;
		std::vector<EPointCategory> m_CategoryMap;
	};
}