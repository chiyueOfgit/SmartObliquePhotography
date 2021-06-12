#include "pch.h"
#include "PointCluster4Score.h"

using namespace hiveObliquePhotography::AutoRetouch;

CPointCluster4Score::CPointCluster4Score(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel) : IPointCluster(vPointIndices, vLabel)
{
	_ASSERTE(vPointIndices != nullptr);
	_ASSERTE(!vPointIndices->empty());
	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
    
	for(auto Index: *vPointIndices)
	{
		m_Position += pCloud->at(Index).getVector3fMap();
		m_Normal += pCloud->at(Index).getNormalVector3fMap();
		m_Color += pCloud->at(Index).getRGBVector3i();
	}
	m_Position /= vPointIndices->size();
	m_Normal /= vPointIndices->size();
	m_Color /= vPointIndices->size();
}

double CPointCluster4Score::computeDistanceV(pcl::index_t vPointIndex) const
{
	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	_ASSERTE(pCloud != nullptr);
	_ASSERTE(vPointIndex < pCloud->size());

	const Eigen::RowVector3f Normal = pCloud->at(vPointIndex).getNormalVector3fMap();
	const Eigen::RowVector3i Color = pCloud->at(vPointIndex).getRGBVector3i();
	const Eigen::RowVector3f Position = pCloud->at(vPointIndex).getVector3fMap();

	double Score = 10000.0;

	Score /= 10.0 * (Position - m_Position).norm() + DBL_EPSILON;
	Score /= (Normal - m_Normal).norm() + DBL_EPSILON;
	Score /= static_cast<double>((Color - m_Color).norm()) + DBL_EPSILON;

	return Score;
}