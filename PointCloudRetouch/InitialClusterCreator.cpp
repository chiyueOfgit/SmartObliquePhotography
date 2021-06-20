#include "pch.h"
#include "InitialClusterCreator.h"
#include "PointCluster.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
CPointCluster* CInitialClusterCreator::createInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, double vRadius, double vHardness, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, const hiveConfig::CHiveConfig *vClusterConfig)
{
	CPointCluster* pInitialCluster = new CPointCluster;

	std::vector<double> PointHardnessSet;
	__generateHardness4EveryPoint(vUserMarkedRegion, vRadius, vHardness, vCameraPos, vPvMatrix, PointHardnessSet);

	pcl::index_t ClusterCenter = __computeClusterCenter(vUserMarkedRegion, PointHardnessSet);

	std::optional<double> DivideThreshold = vClusterConfig->getAttribute<double>("HARDNESS_THRESHOLD");
	if (!DivideThreshold.has_value())
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("The threshold for dividing point set is not defined. The default value [%1%] is employed.", m_DefaultPointSetDivideThreshold));
		DivideThreshold = m_DefaultPointSetDivideThreshold;
	}

	std::vector<pcl::index_t> FeatureGenerationSet, ValidationSet;
	__divideUserSpecifiedRegion(vUserMarkedRegion, PointHardnessSet, DivideThreshold.value(), FeatureGenerationSet, ValidationSet);

	pInitialCluster->init(vClusterConfig, ClusterCenter, FeatureGenerationSet, ValidationSet, CPointCloudRetouchManager::getInstance()->addAndGetTimestamp());

	return pInitialCluster;
}

//*****************************************************************
//FUNCTION: 
void CInitialClusterCreator::__divideUserSpecifiedRegion(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<double> vPointHardnessSet, double vDivideThreshold, std::vector<pcl::index_t>& voFeatureGenerationSet, std::vector<pcl::index_t>& voValidationSet)
{
//TODO：根据阈值和点的hardness，把vUserMarkedRegion分为两个集合
}

//*****************************************************************
//FUNCTION: 
pcl::index_t CInitialClusterCreator::__computeClusterCenter(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<double> vPointHardnessSet)
{
//TODO: 挑选hardness最高的点最为聚类中心
	return 0;
}

//*****************************************************************
//FUNCTION: 
void CInitialClusterCreator::__generateHardness4EveryPoint(const std::vector<pcl::index_t>& vUserMarkedRegion, double vRadius, double vHardness, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, std::vector<double>& voPointHardnessSet)
{
//TODO: 为每个点计算一个hardness
}
