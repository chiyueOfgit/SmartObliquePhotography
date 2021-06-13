#pragma once
#include <queue>

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

			std::vector<IPointCluster*> getGlobalClusterSet(const std::string& vName) const;

			const SBox& getAreaBox() const { return m_BinaryAreaAABB; }

		private:
			CPointClusterSet() = default;

			std::multimap<std::string, IPointCluster*> m_PointClusterMap;

			std::queue<std::vector<std::string>> m_UndoQueue;

			SBox m_BinaryAreaAABB;

			friend class hiveDesignPattern::CSingleton<CPointClusterSet>;
		};
	}
}
