#pragma once
#include "pch.h"

using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;

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

#ifdef _UNIT_TEST
		TColor* data() { return m_Data.data(); };
#endif	
	private:
		Eigen::Matrix<TColor, -1, -1> m_Data;
	};

	template<typename T>
	struct extract_value_type {
	private:
		template <typename _T>
		static auto check(_T) -> typename _T::value_type;
		static void check(...);

	public:
		using type = decltype(check(std::declval<T>()));
	};

	template <class T>
	using extract_value_type_t = typename extract_value_type<T>::type;

	template<typename TColor>
	class CColorTraits
	{
	public:
		using Scalar_t = std::conditional_t<std::is_arithmetic_v<TColor> || std::is_array_v<TColor>, std::remove_extent_t<TColor>, extract_value_type_t<TColor>>;

		constexpr static size_t extractChannel()
		{
			if constexpr (std::is_arithmetic_v<TColor>)
				return 1;
			else if constexpr (std::is_array_v<TColor>)
				return std::extent_v<TColor>;
			else if constexpr (std::is_void_v<Scalar_t>)
				return 0;
			else
				return std::size(TColor());
		}
	};

	template<typename T>
	std::ostream& operator << (std::ostream& vOut, CColorTraits<T>)
	{
		return vOut <<
			"Scalar_t:\t" << typeid(CColorTraits<T>::Scalar_t).name() << '\n' <<
			"Channel:\t" << CColorTraits<T>::extractChannel() << '\n';
	}
}