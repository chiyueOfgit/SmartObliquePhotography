#include "pch.h"
#include "PointLabelChangeRecord.h"
#include "AutoRetouchInterface.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
bool CPointLabelChangeRecord::undoV()
{
	_ASSERTE(!m_ChangeRecord.empty());
	CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet()->undo(m_ChangeRecord);

	if(m_bIsClusterChanged)
	{
		CPointClusterSet::getInstance()->undo();
	}
	return true;
}