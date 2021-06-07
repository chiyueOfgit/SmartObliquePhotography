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
	auto pKdTree = pScene->getGlobalKdTree();

	_ASSERTE(pScene && pCloud && pKdTree);

	static pcl::PointCloud<pcl::PointSurfel>::Ptr pCloudPtr = pCloud->makeShared();
	static pcl::search::KdTree<pcl::PointSurfel>::Ptr pTreePtr = std::make_shared<pcl::search::KdTree<pcl::PointSurfel>>(pKdTree);

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
	Estimation.setInputCloud(pCloudPtr);
	Estimation.setInputNormals(pCloudPtr);
	Estimation.setIndices(Indices);
	Estimation.setSearchMethod(pTreePtr);
	Estimation.compute(Result);

	voVFHDescriptor = Result.empty() ? voVFHDescriptor = Eigen::ArrayXf::Zero(pcl::VFHSignature308::descriptorSize()) :
		voVFHDescriptor = Result.getMatrixXfMap(pcl::VFHSignature308::descriptorSize(), pcl::VFHSignature308::descriptorSize(), 0).col(0);
}