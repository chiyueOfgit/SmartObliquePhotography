#include "pch.h"
#include "PointClusterSet.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

CPointClusterSet::~CPointClusterSet()
{
}

void CPointClusterSet::reset()
{
	for (auto pCluster : m_ClusterSet)
		delete pCluster;

	m_ClusterSet.clear();
}