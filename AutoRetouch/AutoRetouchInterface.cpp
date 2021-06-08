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

bool hiveObliquePhotography::AutoRetouch::hiveExecuteBinaryClassifier(const std::string& vClassifierSig)
{
	_ASSERTE(vClassifierSig.find(CLASSIFIER_BINARY_VFH) != std::string::npos);

	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

	return pClassifier->execute<CBinaryClassifierByVFHAlg>(true, CPointCloudAutoRetouchScene::getInstance()->getPointClusters());
}

bool hiveObliquePhotography::AutoRetouch::hiveExecuteClusterAlg2CreateCluster(const std::vector<int>& vPointIndices, EPointLabel vExpectLabel)
{
	//const std::string ClusterAlgSig = "";
	//IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	//_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", ClusterAlgSig), false);

	//if (pClassifier->execute<CClusterClassifier>(true, vPointIndices, vExpectLabel))
	//{
	//	CPointCloudAutoRetouchScene::getInstance()->addPointCluster(new CPointCluster4VFH(pClassifier->getResultIndices(), vExpectLabel));

	//	return true;
	//}
	//else
	//	return false;

	std::vector<std::uint64_t> Indices(vPointIndices.begin(), vPointIndices.end());
	CPointCloudAutoRetouchScene::getInstance()->addPointCluster(new CPointCluster4VFH(Indices, vExpectLabel));
	return true;
}