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

bool hiveObliquePhotography::AutoRetouch::hiveExecuteCompositeBinaryClassifier()
{
	auto pCompositeClassifier = new CCompositeBinaryClassifierAlg;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pCompositeClassifier->addBinaryClassifiers(COMPOSITE_BINARY_CONFIG);
	pCompositeClassifier->run();
	return true;
}

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
		for (auto& Type : COMPOSITE_BINARY_CONFIG)
		{
			if (Type.find(BINARY_CLUSTER_VFH) != std::string::npos)
			{
				Names.push_back(BINARY_CLUSTER_VFH + std::to_string(ClusterId));
				pClusters.push_back(new CPointCluster4VFH(pClassifier->getResultIndices(), vExpectLabel));
			}
			else if (Type.find(BINARY_CLUSTER_SCORE) != std::string::npos)
			{
				Names.push_back(BINARY_CLUSTER_SCORE + std::to_string(ClusterId));
				pClusters.push_back(new CPointCluster4Score(pClassifier->getResultIndices(), vExpectLabel));
			}
			else if (Type.find(BINARY_CLUSTER_NORMAL) != std::string::npos)
			{
				Names.push_back(BINARY_CLUSTER_NORMAL + std::to_string(ClusterId));
				pClusters.push_back(new CPointCluster4NormalRatio(pClassifier->getResultIndices(), vExpectLabel));
			}
		}
		CPointClusterSet::getInstance()->addPointClusters(Names, pClusters);

		ClusterId++;

		RECORD_TIME_END(添加簇);
		return true;
	}
	else
		return false;
}

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

bool hiveObliquePhotography::AutoRetouch::hiveExecuteMaxVisibilityClustering(const pcl::IndicesPtr& vioPointIndices, EPointLabel vExpectLabel, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix)
{
	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	return pClassifier->execute<CMaxVisibilityClusterAlg>(true, vioPointIndices, vExpectLabel, vCameraPos, vPvMatrix);
}

bool hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowingByColor(const pcl::Indices& vPointIndices, EPointLabel vExpectLabel)
{
	const std::string ClusterAlgSig = CLASSIFIER_REGION_GROW_COLOR;
	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	return pClassifier->execute<CRegionGrowingByColorAlg>(true, vPointIndices, vExpectLabel);
}
