#pragma once
#include "pch.h"

namespace hiveObliquePhotography
{
	using TColor = Eigen::Vector3f;
	class CImage
	{
	public:
		unsigned m_Height = 0;
		unsigned m_Width = 0;
		
		CImage() = default;
		~CImage() = default;

		[[nodiscard]] auto getHeight() const { return m_Height; }
		[[nodiscard]] auto getWidth() const { return m_Width; }
		TColor& fetchColor(unsigned vRowId, unsigned vColId) { return m_Data(vRowId, vColId); }
		const TColor& getColor(unsigned vRowId, unsigned vColId) const { return m_Data(vRowId, vColId); }
		void fillColor(unsigned vHeight, unsigned vWidth, TColor* vBuffer)
		{
			m_Height = vHeight;
			m_Width = vWidth;

			m_Data.resize(m_Height, m_Width);
			std::memcpy(m_Data.data(), vBuffer, m_Data.size() * sizeof(TColor));
		}
	
	private:
		Eigen::Matrix<TColor, -1, -1> m_Data;
	};
}