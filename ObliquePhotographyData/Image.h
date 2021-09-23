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

	template <class T>
	struct has_member_func_size
	{
	private:
		template <class _T> static constexpr auto check(int) -> decltype(&_T::size, bool())
		{
			return std::is_member_function_pointer_v<decltype(&_T::size)>;
		}
		template <class> static constexpr bool check(...) { return false; }
	public:
		static constexpr bool value = check<T>(int());
	};

	template <class T>
	constexpr bool has_member_func_size_v = has_member_func_size<T>::value;

	template<class T>
	struct extract_value_type
	{
	private:
		template <class _T>	static auto check(_T) -> typename _T::value_type;
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
		static_assert(!std::is_void_v<Scalar_t>, "illegal Scalar_t of TColor");

		static constexpr size_t extractChannel()
		{
			if constexpr (std::is_array_v<TColor>)//TODO: 待vs19的C++20实现后，改为std::is_bounded_array<TColor>，注意sizeof(int[])会出错
			{
				constexpr auto Extent = std::extent_v<TColor>;
				static_assert(Extent != 0, "illegal Scalar_t of TColor");
				return Extent;
			}
			else if constexpr (std::has_unique_object_representations_v<TColor>)
				return sizeof(TColor) / sizeof(Scalar_t);
			else if constexpr (has_member_func_size_v<TColor>)
				return std::size(TColor());//may on runtime, may be zero!!!
			else
				static_assert(false, "illegal Channel of TColor");
		}
	};

	template<class TColor>
	std::ostream& operator << (std::ostream& vOut, CColorTraits<TColor>)
	{
		return vOut << std::boolalpha <<
			"Name:     " << typeid(TColor).name() << '\n' <<
			"Memcpy:   " << std::is_trivially_copyable_v<TColor> << '\n' <<
			"SameObj:  " << std::has_unique_object_representations_v<TColor> << '\n' <<
			"HasSize:  " << has_member_func_size_v<TColor> << '\n' <<
			"Scalar_t: " << typeid(CColorTraits<TColor>::Scalar_t).name() << '\n' <<
			"Channel:  " << CColorTraits<TColor>::extractChannel() << '\n';
	}
}