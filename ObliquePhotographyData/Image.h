#pragma once
#include "pch.h"

namespace hiveObliquePhotography
{
	template <class TColor>
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
			m_Data = Eigen::Map<decltype(m_Data)>(vBuffer, vHeight, vWidth);
		}
	
	private:
		Eigen::Matrix<TColor, -1, -1> m_Data;
	};

	template<class T>
	struct extract_value_type
	{
	private:
		template <class _T>
		static auto check(_T) -> typename _T::value_type;
		static void check(...);
	public:
		using type = decltype(check(std::declval<T>()));
	};

	template <class T>
	using extract_value_type_t = typename extract_value_type<T>::type;

	template<class TColor>
	class CColorTraits
	{
	public:
		using Scalar_t = std::conditional_t<std::is_arithmetic_v<TColor> || std::is_array_v<TColor>, std::remove_extent_t<TColor>, extract_value_type_t<TColor>>;

		constexpr static size_t extractChannel()
		{
			if constexpr (std::is_void_v<Scalar_t>)
				return 0;
			else if constexpr (std::has_unique_object_representations_v<TColor>)
				return sizeof(TColor) / sizeof(Scalar_t);
			else
				return std::size(TColor());
		}
	};

	template<class TColor>
	std::ostream& operator << (std::ostream& vOut, CColorTraits<TColor>)
	{
		return vOut << std::boolalpha <<
			"Name:     " << typeid(TColor).name() << '\n' <<
			"Memcpy:   " << std::is_trivially_copyable_v<TColor> << '\n' <<
			"SameObj:  " << std::has_unique_object_representations_v<TColor> << '\n' <<
			"Scalar_t: " << typeid(CColorTraits<TColor>::Scalar_t).name() << '\n' <<
			"Channel:  " << CColorTraits<TColor>::extractChannel() << '\n';
	}
}