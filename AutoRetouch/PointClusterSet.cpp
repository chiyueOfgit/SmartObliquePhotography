#include "pch.h"
#include "PointClusterSet.h"
#include "PointCluster.h"

using namespace hiveObliquePhotography::AutoRetouch;

bool CPointClusterSet::addPointCluster(const std::string& vName, IPointCluster* vCluster)
{
	_ASSERTE(vName != "" && vName != "\n");

	m_PointClusterMap.insert({ vName, vCluster });

	if (vCluster->getClusterLabel() == EPointLabel::UNWANTED)
	{
		m_BinaryAreaAABB.update(vCluster->getClusterAABB());
	}

	return true;
}

bool CPointClusterSet::deletePointCluster(const std::string& vName)
{
	auto Iter = m_PointClusterMap.find(vName);
	if (Iter != m_PointClusterMap.end())
	{
		for (int i = 0; i < m_PointClusterMap.count(vName); i++)
			delete Iter->second;
		m_PointClusterMap.erase(vName);
		return true;
	}
	else
		return false;
}

std::vector<IPointCluster*> CPointClusterSet::getGlobalClusterSet(std::string vName) const
{
	std::vector<IPointCluster*> ClusterSet;

	for (auto& Pair : m_PointClusterMap)
	{
		if (Pair.first.find(vName) != std::string::npos)
		{
			ClusterSet.push_back(Pair.second);
		}
	}

	return ClusterSet;
}
