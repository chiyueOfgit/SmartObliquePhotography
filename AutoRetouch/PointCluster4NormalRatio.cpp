#include "pch.h"
#include "PointCluster4NormalRatio.h"
#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::AutoRetouch;

CPointCluster4NormalRatio::CPointCluster4NormalRatio(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel) : IPointCluster(vPointIndices, vLabel), m_PointIndices(vPointIndices)
{	
	_ASSERTE(vPointIndices != nullptr);
	_ASSERTE(!vPointIndices->empty());
}

//*****************************************************************
//FUNCTION: 
double CPointCluster4NormalRatio::computeSimilarityV(pcl::index_t vPointIndex) const
{
	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	//for (const auto& CurrentIndex : *m_PointIndices)
	//	if (CurrentIndex < 0 || CurrentIndex >= pCloud->size())
	//		_THROW_RUNTIME_ERROR("Out of range");
	if (vPointIndex < 0 || vPointIndex >= pCloud->size())
		_THROW_RUNTIME_ERROR("Index is out of range");

	const auto Threshold = *CAutoRetouchConfig::getInstance()->getAttribute<float>(KEY_WORDS::BINARY_CLASSIFIER_NORMAL_RATIO_THRESHOLD);
	const Eigen::Vector3f PointNormal = pCloud->at(vPointIndex).getNormalVector3fMap();
	int NormalCount = 0;
	for (const auto& CurrentIndex : *m_PointIndices)
		if (PointNormal.dot(pCloud->at(CurrentIndex).getNormalVector3fMap()) >= Threshold)
			++NormalCount;

	return static_cast<double>(NormalCount) / static_cast<double>(m_PointIndices->size());
}
