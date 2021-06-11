#pragma once
#include "AutoRetouchExport.h"
#include "PointCloudAutoRetouchScene.h"
#include "RegionGrowingAlg.h"
#include "RegionGrowingByColorAlg.h"
#include "PointClusterSet.h"
#include "BinaryClassifierAlg.h"
#include "BinaryClassifierAlgByVFH.h"
#include "PointCluster4VFH.h"
#include "SpatialClusteringAlg.h"
#include "MaxVisibilityClusterAlg.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		AUTORETOUCH_DECLSPEC void hiveInitPointCloudScene(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);
		AUTORETOUCH_DECLSPEC void hiveUndoLastOp();

		AUTORETOUCH_DECLSPEC void hiveGetGlobalPointLabelSet(std::vector<EPointLabel>& voGlobalLabel);

		AUTORETOUCH_DECLSPEC bool hiveSwitchPointLabel(EPointLabel vTo, EPointLabel vFrom);

		AUTORETOUCH_DECLSPEC bool hiveExecuteBinaryClassifier(const std::string& vClassifierSig);

		AUTORETOUCH_DECLSPEC bool hiveExecuteClusterAlg2CreateCluster(const std::vector<int>& vPointIndices, EPointLabel vExpectLabel, const pcl::visualization::Camera& vCamera);

		AUTORETOUCH_DECLSPEC bool hiveExecuteClusterAlg2RegionGrowing(const std::vector<int>& vPointIndices, EPointLabel vExpectLabel, const pcl::visualization::Camera& vCamera);

		//FIXME:使用模板会发生重定义
		template<class... TArgs>
		bool hiveExecuteClusteringClassifier(const std::string& vClassifierSig, TArgs&&... vArgs)
		{
			IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
			_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

			return pClassifier->execute<CMaxVisibilityClusterAlg>(true, std::forward<TArgs>(vArgs)...);
		}

		template<class... TArgs>
		bool hiveExecuteRegionGrowClassifier(const std::string& vClassifierSig, TArgs&&... vArgs)
		{
			IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
			_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

			return pClassifier->execute<CRegionGrowingByColorAlg>(true, std::forward<TArgs>(vArgs)...);
		}

		/*template<class... TArgs>
		bool hiveExecuteOutlierDetectionClassifier(const std::string& vClassifierSig, TArgs&&... vArgs)
		{
			IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
			_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

			return pClassifier->execute<CStaOutlierDetectingAlg>(true, std::forward<TArgs>(vArgs)...);
		}*/
	}
}