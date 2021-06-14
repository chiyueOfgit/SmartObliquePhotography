#include "pch.h"
#include "PointCluster4Score.h"

using namespace hiveObliquePhotography::AutoRetouch;

CPointCluster4Score::CPointCluster4Score(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel) : IPointCluster(vPointIndices, vLabel)
{
	_ASSERTE(vPointIndices != nullptr);
	_ASSERTE(!vPointIndices->empty());
	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	Eigen::Vector3f Normal{0,0,0};
	Eigen::Vector3i Color{0,0,0};
	Eigen::Vector3f Position{0,0,0};
	
	for(auto Index: *vPointIndices)
	{
		Normal += pCloud->at(Index).getNormalVector3fMap();
		Color += pCloud->at(Index).getRGBVector3i();
		Position += pCloud->at(Index).getVector3fMap();
	}
	m_Normal = Normal / vPointIndices->size();
	m_Color = Color / vPointIndices->size();
	m_Position = Position / vPointIndices->size();
}

double CPointCluster4Score::computeSimilarityV(pcl::index_t vPointIndex) const
{
	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	_ASSERTE(pCloud != nullptr);
	_ASSERTE(vPointIndex < pCloud->size());

	const Eigen::Vector3f Normal = pCloud->at(vPointIndex).getNormalVector3fMap();
	const Eigen::Vector3i Color = pCloud->at(vPointIndex).getRGBVector3i();
	const Eigen::Vector3f Position = pCloud->at(vPointIndex).getVector3fMap();

	double Score = 10000.0;

	//todo: bug
	//Score /= 10.0 * (Position - m_Position).norm() + DBL_EPSILON;
	//Score /= (Normal - m_Normal).norm() + DBL_EPSILON;
	//Score /= static_cast<double>((Color - m_Color).norm()) + DBL_EPSILON;

	return Score;
}