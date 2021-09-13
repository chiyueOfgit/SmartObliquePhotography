#include "pch.h"
#include "OutlierDetector.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include "PointCloudRetouchManager.h"
#include "common/CpuTimer.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(COutlierDetector, KEYWORD::OUTLIER_DETECTOR)

//*****************************************************************
//FUNCTION: 
void COutlierDetector::runV(pcl::Indices& vInputIndices, EPointLabel vTargetLabel,const float DEV_MUL_THRESH,const int MIN_K, const bool POINT_FILTER_CONDITION)  //FIXME: 实际从vConfig就拿三个值，还不如直接把这三个值作为参数传进来，这样就不依赖CHiveConfig了，你的测试用例也会变得更简单
{
	if (vInputIndices.empty()) return;

	auto pManager = CPointCloudRetouchManager::getInstance();
	for (auto CurrentIndex : vInputIndices)
		if (CurrentIndex < 0 || CurrentIndex >= pManager->getScene().getNumPoint())
			_THROW_RUNTIME_ERROR("Index is out of range");
	
	PointCloud_t::Ptr pCloud(new pcl::PointCloud<pcl::PointSurfel>);
	for (auto Index : vInputIndices)  //FIXME:到这里你实际已经对整个点云做了三次遍历了，第一次获得UNDETERMINED的点的索引，第二次获得KEPT的点的索引，这里是
									  //      第三次遍历。不能在CPointCloudRetouchManager里设计个函数，直接只遍历一次，返回一个PointCloud_t::Ptr吗？
	{
		pcl::PointSurfel TempPoint;
		auto Pos = CPointCloudRetouchManager::getInstance()->getScene().getPositionAt(Index);
		TempPoint.x = Pos.x();
		TempPoint.y = Pos.y();
		TempPoint.z = Pos.z();
		auto Normal = CPointCloudRetouchManager::getInstance()->getScene().getNormalAt(Index);
		TempPoint.normal_x = Normal.x();
		TempPoint.normal_y = Normal.y();
		TempPoint.normal_z = Normal.z();
		auto Color = CPointCloudRetouchManager::getInstance()->getScene().getColorAt(Index);
		TempPoint.r = Color.x();
		TempPoint.g = Color.y();
		TempPoint.b = Color.z();
		TempPoint.curvature = Index;  //FIXME：确定在pcl::RadiusOutlierRemoval中不会使用到点的curvature属性吗？这样做很危险，pcl如果以后真的用到在这个属性，
									  //       这行代码就是一个隐藏很深的bug。PointCloud<T>不是接受的模板吗？用自定义的模板可以吗？直接在自定义的结构中定义index。
									  //       还有就是，pcl::RadiusOutlierRemoval到底需要什么样的点属性才能执行？一定要颜色吗？半径不需要吗？你的测试用例009里面，
		                              //       ground truth的结果哪里来的？你肯定你拿到的是正确结果吗？
		pCloud->push_back(TempPoint);
	}

#ifdef _UNIT_TEST
	hiveCommon::CCPUTimer Timer;
	Timer.start();
#endif // _UNIT_TEST


	PointCloud_t::Ptr pResultCloud(new pcl::PointCloud<pcl::PointSurfel>);
	pcl::StatisticalOutlierRemoval<pcl::PointSurfel> StatisticalOutlier;     //FIXME: 从include的文件来看，pcl提供了多种去除离群点的方法，为什么选当前这种，有过测试吗？
	StatisticalOutlier.setInputCloud(pCloud);
	StatisticalOutlier.setMeanK(MIN_K);
	StatisticalOutlier.setStddevMulThresh(DEV_MUL_THRESH);
	StatisticalOutlier.setNegative(POINT_FILTER_CONDITION);
	StatisticalOutlier.filter(*pResultCloud);

#ifdef _UNIT_TEST
	Timer.stop();
	m_RunTime = Timer.getElapsedTimeInMS();
#endif //_UNIT_TEST

	for (auto& Point : pResultCloud->points)
	   pManager->tagPointLabel(Point.curvature, vTargetLabel, 0, 0);
	
	pManager->recordCurrentStatus();
}

