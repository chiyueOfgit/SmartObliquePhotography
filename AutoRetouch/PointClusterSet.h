#pragma once
#include <stack>

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster;

		class CPointClusterSet : public hiveDesignPattern::CSingleton<CPointClusterSet>
		{
		public:
			~CPointClusterSet() = default;

			bool addPointCluster(const std::string& vName, IPointCluster* vPointCluster);
			bool addPointClusters(const std::vector<std::string>& vNames, const std::vector<IPointCluster*>& vPointClusters);
			bool deletePointCluster(const std::string& vName);
			bool undo();
			bool reset();

			std::vector<IPointCluster*> getGlobalClusterSet(const std::string& vName) const;

			const SBox& getAreaBox() const { return m_BinaryAreaAABB; }

#ifdef _UNIT_TEST
			auto& getPointClusterMap() const { return m_PointClusterMap; }
#endif

		private:
			CPointClusterSet() = default;

			std::multimap<std::string, IPointCluster*> m_PointClusterMap;

			std::stack<std::vector<std::string>> m_UndoQueue;

			SBox m_BinaryAreaAABB;

			friend class hiveDesignPattern::CSingleton<CPointClusterSet>;
		};
	}
}
