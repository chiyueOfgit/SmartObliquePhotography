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
void CPointClusterSet::reset()
{
	for (auto pCluster : m_ClusterSet)
		delete pCluster;

	m_ClusterSet.clear();
}