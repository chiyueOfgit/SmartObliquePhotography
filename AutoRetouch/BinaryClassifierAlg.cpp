#include "pch.h"
#include "BinaryClassifierAlg.h"

using namespace hiveObliquePhotography::AutoRetouch;

//_REGISTER_EXCLUSIVE_PRODUCT(CBinaryClassifierAlg, CLASSIFIER_BINARY)

//*****************************************************************
//FUNCTION: 
void CBinaryClassifierAlg::runV(const std::vector<IPointCluster*>& vInputClusterSet)
{
	_ASSERTE(vInputClusterSet.empty() || (vInputClusterSet.size() > 1));
}