#include "pch.h"
#include "MeshSutureManager.h"
#include "ObliquePhotographyDataInterface.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION:
bool hiveObliquePhotography::SceneReconstruction::CMeshSutureManager::calTileInfo(const PointCloud_t::Ptr vTilePtr, const std::string vName)
{
	double MinX = DBL_MAX;
	double MaxX = 0.0;
	double MinY = DBL_MAX;
	double MaxY = 0.0;
	for (auto it = vTilePtr->begin(); it != vTilePtr->end(); ++it)
	{
		MinX = (it->x < MinX) ? it->x : MinX;
		MaxX = (it->x > MaxX) ? it->x : MaxX;
		MinY = (it->y < MinY) ? it->y : MinY;
		MaxY = (it->y > MaxY) ? it->y : MaxY;
	}

	TileInfo *pTile = new TileInfo();
	pTile->Name = vName;
	pTile->CentrolX = (MinX + MaxX) / 2.0;
	pTile->CentrolY = (MinY + MaxY) / 2.0;
	pTile->Width = MaxX - MinX;
	pTile->Height = MaxY - MinY;

	m_TileInfoSet.push_back(pTile);

	return false;
}

//*****************************************************************
//FUNCTION:
bool hiveObliquePhotography::SceneReconstruction::CMeshSutureManager::calSutureSequence(std::vector<std::pair<string, string>> voSutureNames)
{
	if (!__calAvgTileSize())
		return false;

	int ColNum = ceil((m_MaxCentrolY - m_MinCentrolY) / m_AvgHeight) + 1;
	int RowNum = ceil((m_MaxCentrolX - m_MinCentrolX) / m_AvgWidth) + 1;
	vector<string> TempVec(ColNum);
	m_TileAdjacency.resize(RowNum, TempVec);

	for (auto pTile : m_TileInfoSet)
	{
		m_TileAdjacency[(pTile->CentrolX - m_MinCentrolX) / m_AvgWidth][(pTile->CentrolY - m_MinCentrolY) / m_AvgHeight] = pTile->Name;
	}

	for (int i = 0; i < RowNum; i++)
	{
		for (int k = 0; k < ColNum - 1; k++)
		{
			if (m_TileAdjacency[i][k] != "" && m_TileAdjacency[i][k + 1] != "")
			{
				voSutureNames.push_back(std::make_pair(m_TileAdjacency[i][k], m_TileAdjacency[i][k + 1]));
			}
		}
	}

	for (int i = 0; i < ColNum; i++)
	{
		for (int k = 0; k < RowNum - 1; k++)
		{
			if (m_TileAdjacency[k][i] != "" && m_TileAdjacency[k + 1][i] != "")
			{
				voSutureNames.push_back(std::make_pair(m_TileAdjacency[k][i], m_TileAdjacency[k + 1][i]));
			}
		}
	}

	return true;
}

//*****************************************************************
//FUNCTION:
bool hiveObliquePhotography::SceneReconstruction::CMeshSutureManager::__calAvgTileSize()
{
	for (auto pTile : m_TileInfoSet)
	{
		m_MinCentrolX = (pTile->CentrolX < m_MinCentrolX) ? pTile->CentrolX : m_MinCentrolX;
		m_MaxCentrolX = (pTile->CentrolX > m_MaxCentrolX) ? pTile->CentrolX : m_MaxCentrolX;
		m_MinCentrolY = (pTile->CentrolY < m_MinCentrolY) ? pTile->CentrolY : m_MinCentrolY;
		m_MaxCentrolY = (pTile->CentrolY > m_MaxCentrolY) ? pTile->CentrolY : m_MaxCentrolY;
		m_AvgWidth += pTile->Width;
		m_AvgHeight += pTile->Height;
	}

	m_AvgWidth /= m_TileInfoSet.size();
	m_AvgHeight /= m_TileInfoSet.size();

	for (int i = 0; i < m_TileInfoSet.size() - 1; i++)
	{
		for (int k = i + 1; k < m_TileInfoSet.size(); k++)
		{
			if (abs(m_TileInfoSet[i]->Width - m_TileInfoSet[k]->Width) > m_AvgWidth * m_ErrorRate || abs(m_TileInfoSet[i]->Height - m_TileInfoSet[k]->Height) > m_AvgHeight * m_ErrorRate)
			{
				return false;
			}
		}
	}

	return true;
}
