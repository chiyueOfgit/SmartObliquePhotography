#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CNeighborhood
		{
		public:
			CNeighborhood() = delete;
			CNeighborhood(std::uint64_t vTargetPoint) : m_TargetPoint(vTargetPoint) {}
			CNeighborhood(std::uint64_t vTargetPoint, const std::vector<std::uint64_t>& vRestrictedPointSet) : m_TargetPoint(vTargetPoint) { _ASSERTE(!m_RestrictedSet.empty()); m_RestrictedSet = std::move(vRestrictedPointSet); }
			CNeighborhood(std::uint64_t vTargetPoint, std::vector<std::uint64_t>&& vRestrictedPointSet) : m_TargetPoint(vTargetPoint) { _ASSERTE(!m_RestrictedSet.empty()); m_RestrictedSet = std::move(vRestrictedPointSet); }
			~CNeighborhood() = default;

		private:
			std::uint64_t m_TargetPoint;
			std::vector<std::uint64_t> m_RestrictedSet;
			std::vector<std::uint64_t> m_Neighborhood;
		};
	}
}

