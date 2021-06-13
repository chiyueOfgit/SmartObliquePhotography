#include "pch.h"
#include "AutoRetouchInterface.h"

using namespace hiveObliquePhotography::AutoRetouch;


//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveUndoLastOp()
{
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveInitPointCloudScene(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud)
{
	CPointCloudAutoRetouchScene::getInstance()->init(vPointCloud);
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveGetGlobalPointLabelSet(std::vector<EPointLabel>& voGlobalLabel)
{
	voGlobalLabel = CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet()->getPointLabelSet();
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveSwitchPointLabel(EPointLabel vTo, EPointLabel vFrom)
{
	_ASSERTE(CPointCloudAutoRetouchScene::getInstance() && CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	auto& PointLabelSet = CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet()->fetchPointLabelSet();

	for (auto& PointLabel : PointLabelSet)
		if (PointLabel == vFrom)
			PointLabel = vTo;
	return true;
}

bool hiveObliquePhotography::AutoRetouch::hiveSwitchPointLabel(const pcl::Indices& vPointIndices, EPointLabel vTo)
{
	_ASSERTE(CPointCloudAutoRetouchScene::getInstance() && CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	auto& PointLabelSet = CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet()->fetchPointLabelSet();

	for (auto& PointLabel : vPointIndices)
		PointLabelSet[PointLabel] = vTo;
			
	return true;
}

bool hiveObliquePhotography::AutoRetouch::hiveExecuteBinaryClassifier(const std::string& vClassifierSig)
{
	RECORD_TIME_BEGIN;

	_ASSERTE(vClassifierSig.find(CLASSIFIER_BINARY_VFH) != std::string::npos);

	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

	pClassifier->execute<CBinaryClassifierAlg>(true);
	RECORD_TIME_END(二分类);
	return true;
}

bool hiveObliquePhotography::AutoRetouch::hiveExecuteClusterAlg2CreateCluster(const pcl::IndicesPtr& vioPointIndices, EPointLabel vExpectLabel, const Eigen::Vector3f& vCameraPos, const std::vector<Eigen::Matrix4d>& vMatrices)
{
	RECORD_TIME_BEGIN;
	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	if (pClassifier->execute<CMaxVisibilityClusterAlg>(true, true, vioPointIndices, vExpectLabel, vCameraPos, vMatrices))
	{
		RECORD_TIME_END(聚类);

		RECORD_TIME_BEGIN;

		static std::size_t ClusterId = 0;
		CPointClusterSet::getInstance()->addPointCluster(std::to_string(ClusterId++), new CPointCluster4VFH(pClassifier->getResultIndices(), vExpectLabel));

		RECORD_TIME_END(添加簇);
		return true;
	}
	else
		return false;
}

bool hiveObliquePhotography::AutoRetouch::hiveExecuteClusterAlg2RegionGrowing(const pcl::IndicesPtr& vioPointIndices, EPointLabel vExpectLabel, const Eigen::Vector3f& vCameraPos, const std::vector<Eigen::Matrix4d>& vMatrices)
{
	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	if (pClassifier->execute<CMaxVisibilityClusterAlg>(true, vioPointIndices, vExpectLabel, vCameraPos, vMatrices))
	{
		//TODO: 区域生长是否接收Ptr
		hiveExecuteRegionGrowClassifier( CLASSIFIER_REGION_GROW_COLOR, *vioPointIndices, vExpectLabel);
		return true;
	}
	else
		return false;
}

bool hiveObliquePhotography::AutoRetouch::hiveExecuteMaxVisibilityClustering(const pcl::IndicesPtr& vioPointIndices, EPointLabel vFinalLabel, const Eigen::Vector3f& vCameraPos, const std::vector<Eigen::Matrix4d>& vMatrices)
{
	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	return pClassifier->execute<CMaxVisibilityClusterAlg>(true, vioPointIndices, vFinalLabel, vCameraPos, vMatrices);
}