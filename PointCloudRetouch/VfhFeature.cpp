#include "pch.h"
#include "VfhFeature.h"
#include "PointCloudRetouchManager.h"
#include <pcl/features/impl/vfh.hpp>

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CVfhFeature, KEYWORD::VFH_FEATURE)

CVfhFeature::CVfhFeature()
{
	auto Scene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	auto CloudSize = Scene.getNumPoint();
	_ASSERTE(CloudSize > 0);
	if (!m_pCloud)
	{
		m_pCloud.reset(new PointCloud_t);

		for (int i = 0; i < CloudSize; i++)
		{
			pcl::PointSurfel TempPoint;
			auto Pos = Scene.getPositionAt(i);
			TempPoint.x = Pos.x();
			TempPoint.y = Pos.y();
			TempPoint.z = Pos.z();
			auto Normal = Scene.getNormalAt(i);
			TempPoint.normal_x = Normal.x();
			TempPoint.normal_y = Normal.y();
			TempPoint.normal_z = Normal.z();
			m_pCloud->push_back(TempPoint);
		}
	}
	if (!m_pTree)
	{
		m_pTree.reset(new pcl::search::KdTree<pcl::PointSurfel>);
		m_pTree->setInputCloud(m_pCloud);
	}
}

//*****************************************************************
//FUNCTION: 
double CVfhFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	__computeVfhDescriptor(vDeterminantPointSet, m_DeterminantVfhDescriptor);
	Eigen::Matrix<float, 308, 1> CenterVfhDescriptor;
	__computeVfhDescriptor({ vClusterCenter }, CenterVfhDescriptor);
	m_BaseDotResult = __blockDotVfhDescriptor(CenterVfhDescriptor, m_DeterminantVfhDescriptor, m_BlockSize);
	
	Eigen::Matrix<float, 308, 1> ValidationVfhDescriptor;
	__computeVfhDescriptor(vValidationSet, ValidationVfhDescriptor);
	auto ValidationDotResult = __blockDotVfhDescriptor(ValidationVfhDescriptor, m_DeterminantVfhDescriptor, m_BlockSize);

	return ValidationDotResult / m_BaseDotResult;
}

//*****************************************************************
//FUNCTION: 
double CVfhFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	Eigen::Matrix<float, 308, 1> PointVfhDescriptor;
	__computeVfhDescriptor({ vInputPoint }, PointVfhDescriptor);
	auto PointDotResult = __blockDotVfhDescriptor(PointVfhDescriptor, m_DeterminantVfhDescriptor, m_BlockSize);
	return PointDotResult / m_BaseDotResult;
}

//*****************************************************************
//FUNCTION: 
void CVfhFeature::__computeVfhDescriptor(const std::vector<pcl::index_t>& vPointIndices, Eigen::Matrix<float, 308, 1>& voVfhDescriptor) const
{
	pcl::IndicesPtr pIndices(new pcl::Indices(vPointIndices.begin(), vPointIndices.end()));
	if (vPointIndices.size() == 1)
		pIndices->push_back(pIndices->front());

	pcl::VFHEstimation<std::decay<decltype(*m_pCloud)>::type::PointType, std::decay<decltype(*m_pCloud)>::type::PointType> Estimation;
	pcl::PointCloud<pcl::VFHSignature308> Result;
	Estimation.setInputCloud(m_pCloud);
	Estimation.setInputNormals(m_pCloud);
	Estimation.setIndices(pIndices);
	Estimation.setSearchMethod(m_pTree);
	Estimation.compute(Result);

	voVfhDescriptor = Result.empty() ? voVfhDescriptor = Eigen::ArrayXf::Zero(pcl::VFHSignature308::descriptorSize()) :
		voVfhDescriptor = Result.getMatrixXfMap(pcl::VFHSignature308::descriptorSize(), pcl::VFHSignature308::descriptorSize(), 0).col(0);
}

double CVfhFeature::__blockDotVfhDescriptor(const Eigen::Matrix<float, 308, 1>& vLVfh, const Eigen::Matrix<float, 308, 1>& vRVfh, std::size_t vBlockSize) const
{
	_ASSERTE(vBlockSize > 0 && vBlockSize <= 308);
	if (vBlockSize <= 0)
		vBlockSize = 5;
	else if (vBlockSize > 308)
		vBlockSize = 308;

	double BlockDot = 0.0;
	for (int i = 0; i < 308; i += vBlockSize)
	{
		float LTemp = 0.0f, RTemp = 0.0f;
		for (int j = 0; j < vBlockSize && (i + j) < 308; j++)
		{
			LTemp += vLVfh.row(i + j).value();
			RTemp += vRVfh.row(i + j).value();
		}
		BlockDot += LTemp * RTemp;
	}

	return BlockDot;
}
