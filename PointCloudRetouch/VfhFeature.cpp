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
	_ASSERTE(m_pConfig);
	{
		std::optional<int> KernelSize = m_pConfig->getAttribute<int>("CONVOLUTION_KERNEL_SIZE");
		if (KernelSize.has_value())
			m_KernelSize = KernelSize.value();
	}

	if (vDeterminantPointSet.empty())
		return 0.0;

	__computeVfhDescriptor(vDeterminantPointSet, m_DeterminantVfhDescriptor);
	m_BaseDotResult = __KernelDotVfhDescriptor(m_DeterminantVfhDescriptor, m_DeterminantVfhDescriptor, m_KernelSize);
	
	Eigen::Matrix<float, VfhDimension, 1> ValidationVfhDescriptor;
	__computeVfhDescriptor(vValidationSet, ValidationVfhDescriptor);
	auto ValidationDotResult = __KernelDotVfhDescriptor(ValidationVfhDescriptor, m_DeterminantVfhDescriptor, m_KernelSize);

	_ASSERTE(m_BaseDotResult > 0);
	double ValidationRate = ValidationDotResult / m_BaseDotResult;
	return ValidationRate > 1.0 ? 1.0 : ValidationRate;
}

//*****************************************************************
//FUNCTION: 
double CVfhFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	Eigen::Matrix<float, VfhDimension, 1> PointVfhDescriptor;
	__computeVfhDescriptor({ vInputPoint }, PointVfhDescriptor);
	auto PointDotResult = __KernelDotVfhDescriptor(PointVfhDescriptor, m_DeterminantVfhDescriptor, m_KernelSize);
	double PointRate = PointDotResult / m_BaseDotResult;
	return PointRate > 1.0 ? 1.0 : PointRate;
}

//*****************************************************************
//FUNCTION: 
void CVfhFeature::__computeVfhDescriptor(const std::vector<pcl::index_t>& vPointIndices, Eigen::Matrix<float, VfhDimension, 1>& voVfhDescriptor) const
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

//*****************************************************************
//FUNCTION: 
double CVfhFeature::__KernelDotVfhDescriptor(const Eigen::Matrix<float, VfhDimension, 1>& vLVfh, const Eigen::Matrix<float, VfhDimension, 1>& vRVfh, std::size_t vBlockSize) const
{
	if (vBlockSize <= 0)
		vBlockSize = 5;
	else if (vBlockSize > VfhDimension)
		vBlockSize = VfhDimension;

	double BlockDot = 0.0;
	for (int i = 0; i < VfhDimension; i += vBlockSize)
	{
		float LTemp = 0.0f, RTemp = 0.0f;
		for (int j = 0; j < vBlockSize && (i + j) < VfhDimension; j++)
		{
			LTemp += vLVfh.row(i + j).value();
			RTemp += vRVfh.row(i + j).value();
		}
		BlockDot += LTemp * RTemp;
	}

	return BlockDot;
}
