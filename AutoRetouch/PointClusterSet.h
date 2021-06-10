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

			std::vector<IPointCluster*> getGlobalClusterSet() const;

			const SBox& getAreaBox() const { return m_BinaryAreaAABB; }

		private:
			CPointClusterSet() = default;

			std::map<std::string, IPointCluster*> m_PointClusterMap;

			SBox m_BinaryAreaAABB;

			friend class hiveDesignPattern::CSingleton<CPointClusterSet>;
		};
	}
}
