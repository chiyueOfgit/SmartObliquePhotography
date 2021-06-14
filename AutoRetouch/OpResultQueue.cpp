#include "pch.h"
#include "OpResultQueue.h"
#include "OpResult.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
bool COpResultQueue::undo()
{
	if (m_OpResultQueue.empty()) return false;

	IOpResult* pResult = m_OpResultQueue.back();
	_ASSERTE(pResult);
	pResult->undoV();
	m_OpResultQueue.pop_back();

	return true;
}

void hiveObliquePhotography::AutoRetouch::COpResultQueue::clear()
{
	for (auto pResult : m_OpResultQueue)
		delete pResult;
	m_OpResultQueue.clear();
}
