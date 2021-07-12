#include "pch.h"
#include "NormalComplexity.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CNormalComplexity, KEYWORD::NORMAL_COMPLEXITY)


//*****************************************************************
//FUNCTION: 
double CNormalComplexity::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	return 0.0;
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	return 0.0;
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::__calcSinglePointNormalComplexity(pcl::index_t vIndex, PointCloud_t::Ptr vCloud)
{
	return 1.0;
}
