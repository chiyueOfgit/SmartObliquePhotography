#include "pch.h"
#include "PointLabelChangeRecord.h"

#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
bool CPointLabelChangeRecord::undoV()
{
	//return CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	for(const auto& OneChange:m_ChangeRecord)
	{
		pcl::Indices Indices;
		Indices.push_back(OneChange.Index);
		hiveSwitchPointLabel(Indices, OneChange.SrcLabel);
	}
	return true;
}