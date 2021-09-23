#include "pch.h"
#include "PointClusterSet.h"
#include "PointCluster.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

CPointClusterSet::~CPointClusterSet()
{
	reset();
}

//*****************************************************************
//FUNCTION: 
void CPointClusterSet::removeLastCluster()
{
	if (!m_ClusterSet.empty())
	{
		delete m_ClusterSet.back();
		m_ClusterSet.pop_back();
	}
}

//*****************************************************************
//FUNCTION: 
void CPointClusterSet::removeClustersByLabel(EPointLabel vLabel)
{
	for (auto Iter = m_ClusterSet.begin(); Iter != m_ClusterSet.end();)
	{
		if ((*Iter)->getLabel() == vLabel)
			Iter = m_ClusterSet.erase(Iter);
		else
			Iter++;
	}
}

//*****************************************************************
//FUNCTION: 
void CPointClusterSet::reset()
{
	for (auto pCluster : m_ClusterSet)
		delete pCluster;

	m_ClusterSet.clear();
}