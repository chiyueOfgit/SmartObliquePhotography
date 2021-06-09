#pragma once
#include "AutoRetouchExport.h"
#include "PointCloudAutoRetouchScene.h"
#include "RegionGrowingAlg.h"
#include "BinaryClassifierAlg.h"
#include "SpatialClusteringAlg.h"
#include "MaxVisibilityClusterAlg.h"
#include "BinaryClassifierAlgByVFH.h"
#include "PointCluster4VFH.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		AUTORETOUCH_DECLSPEC void hiveInitPointCloudScene(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);
		AUTORETOUCH_DECLSPEC void hiveUndoLastOp();

		AUTORETOUCH_DECLSPEC void hiveGetGlobalPointLabelSet(std::vector<EPointLabel>& voGlobalLabel);

		template<class... TArgs>
		bool hiveExecuteClassifer(const std::string& vClassifierSig, TArgs&&... vArgs)
		{
			IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
			_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

			return pClassifier->execute<CRegionGrowingAlg>(true, std::forward<TArgs>(vArgs)...);
		}
		
		AUTORETOUCH_DECLSPEC bool hiveExecuteBinaryClassifier(const std::string& vClassifierSig);
		AUTORETOUCH_DECLSPEC bool hiveObliquePhotography::AutoRetouch::hiveExecuteClusterAlg2CreateCluster(const std::vector<int>& vPointIndices, EPointLabel vExpectLabel);

		template<class... TArgs>
		bool hiveExecuteClusteringClassifier(const std::string& vClassifierSig, TArgs&&... vArgs)
		{
			IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
			_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

			return pClassifier->execute<CMaxVisibilityClusterAlg>(true, std::forward<TArgs>(vArgs)...);
		}
	}
}