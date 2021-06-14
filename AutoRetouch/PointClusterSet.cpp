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
	
	m_UndoQueue.push({ vName });
	return true;
}

bool CPointClusterSet::addPointClusters(const std::vector<std::string>& vNames, const std::vector<IPointCluster*>& vPointClusters)
{
	_ASSERTE(vNames.size() == vPointClusters.size());
	if (vNames.size() != vPointClusters.size())
		_THROW_RUNTIME_ERROR("paramerter error.");
	for (int i = 0; i < vNames.size(); i++)
	{
		m_PointClusterMap.insert({ vNames[i], vPointClusters[i] });

		if (vPointClusters[i]->getClusterLabel() == EPointLabel::UNWANTED)
		{
			m_BinaryAreaAABB.update(vPointClusters[i]->getClusterAABB());
		}
	}

	m_UndoQueue.push(vNames);
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

bool CPointClusterSet::undo()
{
	if (!m_UndoQueue.empty())
	{
		auto& Clusters2Delete = m_UndoQueue.top();

		for (auto& Name : Clusters2Delete)
		{
			deletePointCluster(Name);
		}

		m_BinaryAreaAABB.reset();
		for (auto& Pair : m_PointClusterMap)
		{
			if (Pair.second->getClusterLabel() == EPointLabel::UNWANTED)
				m_BinaryAreaAABB.update(Pair.second->getClusterAABB());
		}

		m_UndoQueue.pop();
		return true;
	}
	else
		return false;

}

bool CPointClusterSet::reset()
{
	for (auto Pair : m_PointClusterMap)
		delete Pair.second;
	m_PointClusterMap.clear();

	while (!m_UndoQueue.empty())
		m_UndoQueue.pop();

	m_BinaryAreaAABB.reset();

	return true;
}

std::vector<IPointCluster*> CPointClusterSet::getGlobalClusterSet(const std::string& vName) const
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
