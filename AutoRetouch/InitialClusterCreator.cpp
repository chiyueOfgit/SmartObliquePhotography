#include "pch.h"
#include "InitialClusterCreator.h"
#include "PointCluster.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
void IInitialClusterCreator::runV(const pcl::Indices& vUserInputSet, EPointLabel vLabel, double vRadius)
{
	IPointCluster* pInitialCluster = __createInitialClusterV(vUserInputSet, vRadius);
}