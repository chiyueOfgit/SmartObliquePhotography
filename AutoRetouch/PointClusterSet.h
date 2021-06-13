#pragma once

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
			
			bool deletePointCluster(const std::string& vName);
			bool deletePointCluster();
			
			std::vector<IPointCluster*> getGlobalClusterSet() const;

			std::vector<IPointCluster*> getGlobalClusterSet(std::string vName) const;

			const SBox& getAreaBox() const { return m_BinaryAreaAABB; }

		private:
			CPointClusterSet() = default;

			std::multimap<std::string, IPointCluster*> m_PointClusterMap;

			SBox m_BinaryAreaAABB;

			friend class hiveDesignPattern::CSingleton<CPointClusterSet>;
		};
	}
}
