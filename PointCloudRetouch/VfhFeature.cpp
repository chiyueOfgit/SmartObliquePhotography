#include "pch.h"
#include "VfhFeature.h"
#include <pcl/features/impl/vfh.hpp>

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
double CVfhFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	__computeVfhDescriptor(vDeterminantPointSet, m_VFHDescriptor);

	__computeVfhDescriptor()
}

//*****************************************************************
//FUNCTION: 
double CVfhFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{

}

//*****************************************************************
//FUNCTION: 
void CVfhFeature::__computeVfhDescriptor(const std::vector<pcl::index_t>& vPointIndices, Eigen::Matrix<float, 308, 1>& voVfhDescriptor) const
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

	voVfhDescriptor = Result.empty() ? voVfhDescriptor = Eigen::ArrayXf::Zero(pcl::VFHSignature308::descriptorSize()) :
		voVfhDescriptor = Result.getMatrixXfMap(pcl::VFHSignature308::descriptorSize(), pcl::VFHSignature308::descriptorSize(), 0).col(0);
}