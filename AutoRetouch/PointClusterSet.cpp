#include "pch.h"
#include "PointClusterSet.h"
#include "PointCluster.h"

using namespace hiveObliquePhotography::AutoRetouch;

bool CPointClusterSet::addPointCluster(const std::string& vName, IPointCluster* vCluster)
{
	_ASSERTE(vName != "" && vName != "\n");

	if (m_PointClusterMap.find(vName) == m_PointClusterMap.end())
	{
		m_PointClusterMap[vName] = vCluster;

		if (vCluster->getClusterLabel() == EPointLabel::UNWANTED)
		{
			m_BinaryAreaAABB.update(vCluster->getClusterAABB());
		}

		return true;
	}
	else
		return false;
}

bool CPointClusterSet::deletePointCluster(const std::string& vName)
{
	if (m_PointClusterMap.find(vName) != m_PointClusterMap.end())
	{
		m_PointClusterMap.erase(m_PointClusterMap.find(vName));
		return true;
	}
	else
		return false;
}

std::vector<IPointCluster*> CPointClusterSet::getGlobalClusterSet() const
{
	std::vector<IPointCluster*> ClusterSet;

	for (auto& Pair : m_PointClusterMap)
		ClusterSet.push_back(Pair.second);

	return ClusterSet;
}
