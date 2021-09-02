#pragma once
#include "pch.h"

namespace hiveObliquePhotography
{
	template <typename TColor>
	class CImage
	{
	public:
		CImage() = default;
		~CImage() = default;

		[[nodiscard]] auto getHeight() const { return m_Data.rows(); }
		[[nodiscard]] auto getWidth() const { return m_Data.cols(); }
		
		[[nodiscard]] TColor& fetchColor(unsigned vRowId, unsigned vColId) { return m_Data(vRowId, vColId); }
		[[nodiscard]] const TColor& getColor(unsigned vRowId, unsigned vColId) const { return m_Data(vRowId, vColId); }
		
		void fillColor(unsigned vHeight, unsigned vWidth, TColor* vBuffer)
		{
			m_Data = Eigen::Map<Eigen::Matrix<TColor, -1, -1>>(vBuffer, vHeight, vWidth);
		}
	
	private:
		Eigen::Matrix<TColor, -1, -1> m_Data;
	};
}