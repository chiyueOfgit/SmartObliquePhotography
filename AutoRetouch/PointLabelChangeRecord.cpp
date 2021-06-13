#include "pch.h"
#include "PointLabelChangeRecord.h"
#include "AutoRetouchInterface.h"


using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
bool CPointLabelChangeRecord::undoV()
{
	for(const auto& OneChange:m_ChangeRecord)
	{
		pcl::Indices Indices;
		Indices.push_back(OneChange.Index);
		hiveSwitchPointLabel(Indices, OneChange.SrcLabel);
	}
	if(m_bIsClusterChanged)
	{
		CPointClusterSet::getInstance()->deletePointCluster();
	}
	return true;
}