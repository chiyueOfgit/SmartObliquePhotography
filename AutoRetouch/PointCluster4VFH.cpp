#include "pch.h"
#include "PointCluster4VFH.h"
#include "PointCloudAutoRetouchScene.h"
#include <pcl/features/impl/vfh.hpp>
#include <pcl/filters/impl/extract_indices.hpp>

using namespace hiveObliquePhotography::AutoRetouch;

CPointCluster4VFH::CPointCluster4VFH(const std::vector<std::uint64_t>& vPointIndices, EPointLabel vLabel) : IPointCluster(vLabel)
{
	_ASSERTE(!vPointIndices.empty());
	__computeVFHDescriptor(vPointIndices, m_VFHDescriptor);
	m_PointIndices = std::set<std::uint64_t>(vPointIndices.begin(), vPointIndices.end());
	
	auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();

	SBox AABB;
	for (auto Index : m_PointIndices)
		AABB.update((*pCloud)[Index].x, (*pCloud)[Index].y, (*pCloud)[Index].z);
	setClusterAABB(AABB);
}

//*****************************************************************
//FUNCTION: 
double CPointCluster4VFH::computeDistanceV(std::uint64_t vPointIndex) const
{
	_ASSERTE(vPointIndex < CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene()->size());

	Eigen::Matrix<float, 308, 1> PointDescriptor;
	__computeVFHDescriptor({ vPointIndex }, PointDescriptor);

	float Dot = m_VFHDescriptor.dot(PointDescriptor);

	return Dot;
}

//*****************************************************************
//FUNCTION: 
void CPointCluster4VFH::__computeVFHDescriptor(const std::vector<std::uint64_t>& vPointIndices, Eigen::Matrix<float, 308, 1>& voVFHDescriptor) const
{
	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	auto pCloud = pScene->getPointCloudScene();
	auto pTree = pScene->getGlobalKdTree();

	_ASSERTE(pScene && pCloud && pTree);

	std::set<std::uint64_t> IndicesUnique(vPointIndices.begin(), vPointIndices.end());
	_ASSERTE(IndicesUnique.size() == vPointIndices.size());

	std::vector<std::uint64_t> PointIndicesSorted(vPointIndices.begin(), vPointIndices.end());
	std::sort(PointIndicesSorted.begin(), PointIndicesSorted.end());
	_ASSERTE(vPointIndices.back() < pCloud->size());

	pcl::IndicesPtr Indices(new pcl::Indices(vPointIndices.begin(), vPointIndices.end()));
	if (vPointIndices.size() == 1)
		Indices->push_back(Indices->front());

	pcl::VFHEstimation<pcl::PointSurfel, pcl::PointSurfel> Estimation;
	pcl::PointCloud<pcl::VFHSignature308> Result;
	Estimation.setInputCloud(pCloud);
	Estimation.setInputNormals(pCloud);
	Estimation.setIndices(Indices);
	Estimation.setSearchMethod(pTree);
	Estimation.compute(Result);

	voVFHDescriptor = Result.empty() ? voVFHDescriptor = Eigen::ArrayXf::Zero(pcl::VFHSignature308::descriptorSize()) :
		voVFHDescriptor = Result.getMatrixXfMap(pcl::VFHSignature308::descriptorSize(), pcl::VFHSignature308::descriptorSize(), 0).col(0);
}