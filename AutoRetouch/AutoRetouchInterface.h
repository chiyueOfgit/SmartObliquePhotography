#pragma once
#include "AutoRetouchExport.h"
#include "PointCloudAutoRetouchScene.h"
#include "RegionGrowingAlg.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		AUTORETOUCH_DECLSPEC void hiveInitPointCloudScene(pcl::PointCloud<pcl::PointSurfel>* vPointCloud);
		AUTORETOUCH_DECLSPEC void hiveUndoLastOp();

		template<class... TArgs>
		bool hiveExecuteClassifer(const std::string& vClassifierSig, TArgs&&... vArgs)
		{
			IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(vClassifierSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
			_HIVE_EARLY_RETURN(!pClassifier, _FORMAT_STR1("Fail to execute classifier [%1%] due to unknown classifier signature.", vClassifierSig), false);

			return pClassifier->execute<CRegionGrowingAlg>(true, std::forward<TArgs>(vArgs)...);
		}
	}
}