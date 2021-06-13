#include "pch.h"
#include "CompositeBinaryClassifierAlg.h"

using namespace hiveObliquePhotography::AutoRetouch;

EPointLabel CCompositeBinaryClassifierAlg::__ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const
{
	return EPointLabel::UNWANTED;
}