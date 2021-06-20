#include "pch.h"
#include "PointLabelSet.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::tagPointLabel(const std::vector<pcl::index_t>& vTargetPointSet, EPointLabel vTargetLabel)
{

}

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::init(std::size_t vSize)
{
	_ASSERTE(m_LabelSet.empty() && (vSize > 0));
	m_LabelSet.reserve(vSize);
	for (auto i = 0; i < vSize; i++) m_LabelSet.emplace_back(EPointLabel::UNDETERMINED);
}