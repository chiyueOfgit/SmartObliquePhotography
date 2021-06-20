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
void hiveObliquePhotography::AutoRetouch::hiveInitPointCloudScene(PointCloud_t::Ptr vPointCloud)
{
	CPointCloudAutoRetouchScene::getInstance()->init(vPointCloud);
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveGetPointCloudForSave(PointCloud_t::Ptr voPointCloud)
{
	pcl::Indices PointIndices;
	hiveGetIndicesByLabel(PointIndices, EPointLabel::KEPT);
	hiveGetIndicesByLabel(PointIndices, EPointLabel::UNDETERMINED);
	for (auto Index : PointIndices)
		voPointCloud->push_back(CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene()->at(Index));
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveGetGlobalPointLabelSet(std::vector<EPointLabel>& voGlobalLabel)
{
	voGlobalLabel = CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet()->getPointLabelSet();
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveGetIndicesByLabel(pcl::Indices& voPointIndices, EPointLabel vExpectLabel)
{
	std::vector<EPointLabel> GlobalLabel;
	hiveGetGlobalPointLabelSet(GlobalLabel);
	for (size_t i = 0;i < GlobalLabel.size();i++)
		if (GlobalLabel[i] == vExpectLabel)
			voPointIndices.push_back(i);
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveResetSceneSelectStatus()
{
	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	auto pCluster = CPointClusterSet::getInstance();

	pScene->resetLabelSet();
	pScene->clearResultQueue();
	pCluster->reset();
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveSwitchPointLabel(EPointLabel vTo, EPointLabel vFrom)
{
	_ASSERTE(CPointCloudAutoRetouchScene::getInstance() && CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	return CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet()->switchLabel(vTo, vFrom);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveGetAutoRetouchConfig(CAutoRetouchConfig*& voConfig)
{
	auto pConfig = CAutoRetouchConfig::getInstance();
	_ASSERTE(pConfig);
	if (pConfig)
	{
		voConfig = CAutoRetouchConfig::getInstance();
		return true;
	}
	else
	{
		voConfig = nullptr;
		return false;
	}
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveExecuteBinaryClassifier(const std::string& vClassifierSig, const std::string& vClusterType)
{
	RECORD_TIME_BEGIN;
	
	if (vClassifierSig.find(CLASSIFIER_BINARY) == std::string::npos)
		return false;

	IPointClassifier* pClassifier = hiveDesignPattern::hiveCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

	pClassifier->execute<CBinaryClassifierAlg>(true, vClusterType);
	RECORD_TIME_END(二分类);
	return true;
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveExecuteCompositeBinaryClassifier()
{
	auto pCompositeClassifier = new CCompositeBinaryClassifierAlg;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	auto pConfig = CAutoRetouchConfig::getInstance();
	std::vector<std::string> ClusterTypes;
	if (pConfig->getAttribute<bool>(KEY_WORDS::ENABLE_VFH).value())
		ClusterTypes.push_back(BINARY_CLUSTER_VFH);
	if (pConfig->getAttribute<bool>(KEY_WORDS::ENABLE_SCORE).value())
		ClusterTypes.push_back(BINARY_CLUSTER_SCORE);
	if (pConfig->getAttribute<bool>(KEY_WORDS::ENABLE_NORMAL).value())
		ClusterTypes.push_back(BINARY_CLUSTER_NORMAL);
	pCompositeClassifier->addBinaryClassifiers(ClusterTypes);
	pCompositeClassifier->run();
	return true;
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveExecuteClusterAlg2CreateCluster(const pcl::IndicesPtr& vioPointIndices, EPointLabel vExpectLabel, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix)
{
	RECORD_TIME_BEGIN;
	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	if (pClassifier->execute<CMaxVisibilityClusterAlg>(true, true, vioPointIndices, vExpectLabel, vCameraPos, vPvMatrix))
	{
		RECORD_TIME_END(聚类);

		RECORD_TIME_BEGIN;

		std::vector<std::string> Names;
		std::vector<IPointCluster*> pClusters;

		static std::size_t ClusterId = 0;
		auto pConfig = CAutoRetouchConfig::getInstance();
		if (pConfig->getAttribute<bool>(KEY_WORDS::ENABLE_VFH).value())
		{
			Names.push_back(BINARY_CLUSTER_VFH + std::to_string(ClusterId));
			pClusters.push_back(new CPointCluster4VFH(pClassifier->getResultIndices(), vExpectLabel));
		}
		if (pConfig->getAttribute<bool>(KEY_WORDS::ENABLE_SCORE).value())
		{
			Names.push_back(BINARY_CLUSTER_SCORE + std::to_string(ClusterId));
			pClusters.push_back(new CPointCluster4Score(pClassifier->getResultIndices(), vExpectLabel));
		}
		if (pConfig->getAttribute<bool>(KEY_WORDS::ENABLE_NORMAL).value())
		{
			Names.push_back(BINARY_CLUSTER_NORMAL + std::to_string(ClusterId));
			pClusters.push_back(new CPointCluster4NormalRatio(pClassifier->getResultIndices(), vExpectLabel));
		}

		CPointClusterSet::getInstance()->addPointClusters(Names, pClusters);

		ClusterId++;

		RECORD_TIME_END(添加簇);
		return true;
	}
	else
		return false;
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveExecuteCompositeClusterAndGrowing(const pcl::IndicesPtr& vioPointIndices, EPointLabel vExpectLabel, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix)
{
	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClusterClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClusterClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	const std::string GrowingAlgSig = CLASSIFIER_REGION_GROW_COLOR;
	IPointClassifier* pGrowingClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(GrowingAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pGrowingClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", GrowingAlgSig), false);

	CCompositeClassifier* pCompositeClassifier = new CCompositeClassifier;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pCompositeClassifier->addClassifierAndExecute<CMaxVisibilityClusterAlg>(dynamic_cast<CMaxVisibilityClusterAlg*>(pClusterClassifier), vioPointIndices, vExpectLabel, vCameraPos, vPvMatrix);
	pCompositeClassifier->addClassifierAndExecuteByLastIndices<CRegionGrowingByColorAlg>(dynamic_cast<CRegionGrowingByColorAlg*>(pGrowingClassifier), vExpectLabel);
	pCompositeClassifier->ensembleResult();
	return true;
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveExecuteMaxVisibilityClustering(const pcl::IndicesPtr& vioPointIndices, EPointLabel vExpectLabel, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix)
{
	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	return pClassifier->execute<CMaxVisibilityClusterAlg>(true, vioPointIndices, vExpectLabel, vCameraPos, vPvMatrix);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowingByColor(const pcl::Indices& vPointIndices, EPointLabel vExpectLabel)
{
	const std::string ClusterAlgSig = CLASSIFIER_REGION_GROW_COLOR;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	return pClassifier->execute<CRegionGrowingByColorAlg>(true, vPointIndices, vExpectLabel);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveExecuteOutlierDetecting(pcl::Indices& vioPointIndices, EPointLabel vExpectLabel)
{
	const std::string ClusterAlgSig = CLASSIFIER_OUTLIER_DETECTION;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	return pClassifier->execute<COutlierDetectingAlg>(true, vioPointIndices, vExpectLabel);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveInitBackgroundMarker(const hiveConfig::CHiveConfig* vBackgroundMarkerConfig)
{
	_ASSERTE(vBackgroundMarkerConfig);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveInitLitterMarker(const hiveConfig::CHiveConfig* vLitterMarkerConfig)
{
	_ASSERTE(vLitterMarkerConfig);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveMarkBackground(const pcl::IndicesPtr& vMarkedRegion, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix)
{

}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::AutoRetouch::hiveMarkLitter(const pcl::IndicesPtr& vMarkedRegion, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix)
{

}