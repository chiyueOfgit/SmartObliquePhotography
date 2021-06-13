#include "pch.h"
#include "CompositeBinaryClassifierAlg.h"
#include "BinaryClassifierAlg.h"
#include "PointCluster4VFH.h"
#include "PointCluster4Score.h"
#include "PointCluster4NormalRatio.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
void CCompositeBinaryClassifierAlg::addBinaryClassifiers(const std::vector<std::string>& vClusterTypes)
{
	for (auto& Type : vClusterTypes)
	{
		if (Type.find(BINARY_CLUSTER_VFH) != std::string::npos || Type.find(BINARY_CLUSTER_SCORE) != std::string::npos || Type.find(BINARY_CLUSTER_NORMAL) != std::string::npos)
		{
			addClassifierAndExecute<CBinaryClassifierAlg>(new CBinaryClassifierAlg, Type);
		}
	}
}

//*****************************************************************
//FUNCTION: 
EPointLabel CCompositeBinaryClassifierAlg::__ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const
{
	return EPointLabel::UNWANTED;
}
