#include "pch.h"
#include "PointLabelChangeRecord.h"
#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
bool CPointLabelChangeRecord::undoV()
{
	return CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
}