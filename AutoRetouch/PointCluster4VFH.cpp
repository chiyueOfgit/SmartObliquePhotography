#include "pch.h"
#include "PointCluster4VFH.h"
#include "PointCloudAutoRetouchScene.h"
#include <pcl/features/impl/vfh.hpp>

using namespace hiveObliquePhotography::AutoRetouch;

CPointCluster4VFH::CPointCluster4VFH(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel) : IPointCluster(vPointIndices, vLabel)
{
	_ASSERTE(!vPointIndices->empty());
	__computeVFHDescriptor(*vPointIndices, m_VFHDescriptor);
}

//*****************************************************************
//FUNCTION: 
double CPointCluster4VFH::computeSimilarityV(pcl::index_t vPointIndex) const
{
	_ASSERTE(vPointIndex < CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene()->size());

	Eigen::Matrix<float, 308, 1> PointDescriptor;
	__computeVFHDescriptor({ vPointIndex }, PointDescriptor);

	float Dot = m_VFHDescriptor.dot(PointDescriptor);

	return Dot;
}

//*****************************************************************
//FUNCTION: 
void CPointCluster4VFH::__computeVFHDescriptor(const pcl::Indices& vPointIndices, Eigen::Matrix<float, 308, 1>& voVFHDescriptor) const
{
	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	auto pCloud = pScene->getPointCloudScene();
	auto pTree = pScene->getGlobalKdTree();

	std::set IndicesUnique(vPointIndices.begin(), vPointIndices.end());
	_ASSERTE(IndicesUnique.size() == vPointIndices.size());
	_ASSERTE(*std::max_element(vPointIndices.begin(), vPointIndices.end()) < pCloud->size());

	//TODO: 需要接收pcl::IndicesPtr而非const pcl::Indices&
	pcl::IndicesPtr Indices(new pcl::Indices(vPointIndices.begin(), vPointIndices.end()));
	if (vPointIndices.size() == 1)
		Indices->push_back(Indices->front());

	pcl::VFHEstimation<std::decay<decltype(*pCloud)>::type::PointType, std::decay<decltype(*pCloud)>::type::PointType> Estimation;
	pcl::PointCloud<pcl::VFHSignature308> Result;
	Estimation.setInputCloud(pCloud);
	Estimation.setInputNormals(pCloud);
	Estimation.setIndices(Indices);
	Estimation.setSearchMethod(pTree);
	Estimation.compute(Result);

	voVFHDescriptor = Result.empty() ? voVFHDescriptor = Eigen::ArrayXf::Zero(pcl::VFHSignature308::descriptorSize()) :
		voVFHDescriptor = Result.getMatrixXfMap(pcl::VFHSignature308::descriptorSize(), pcl::VFHSignature308::descriptorSize(), 0).col(0);
}