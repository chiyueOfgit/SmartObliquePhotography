#include "pch.h"
#include "UnorganziedPointCloud2MultiLayeredConverter.h"
#include "MultiLayeredPointCloud.h"
#include "OverallConfig.h"
#include "PointCloudFileLoader.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
CMultiLayeredPointCloud* CUnorganziedPointCloud2MultiLayeredConverter::convertUnorganizedPointCloud2MultiLayered()
{
	_ASSERTE(COverallConfig::getInstance()->isReady() && !m_pMultiLayeredPointCloud && m_ActiveTaskSet.empty());

	m_ActiveTaskSet.resize(COverallConfig::getInstance()->getAttribute<std::uint8_t>(Keywords::MAX_NUM_POINT_CLOUD_CONVERT_TASK).value());

	try
	{
		m_pMultiLayeredPointCloud = new CMultiLayeredPointCloud;
		m_pMultiLayeredPointCloud->init(COverallConfig::getInstance()->getAttribute<float>(Keywords::SCENE_LONGITUDE_SPAN).value(), COverallConfig::getInstance()->getAttribute<float>(Keywords::SCENE_LATITUDE_SPAN).value(),
			COverallConfig::getInstance()->getAttribute<float>(Keywords::CELL_SCALE).value(), COverallConfig::getInstance()->getAttribute<unsigned char>(Keywords::EXPECTED_BLOCK_SIZE_IN_MB).value());

		std::string FileName;
		std::uint32_t TileX, TileY;
		std::tie(FileName, TileX, TileY) = COverallConfig::getInstance()->getFirstPointCloudFile();
		while (!FileName.empty())
		{
			SActiveTask& Task = __updateTaskStatus();
			Task._FileName = FileName;
			Task._TileX = TileX;
			Task._TileY = TileY;
			Task._Result = std::async(&CUnorganziedPointCloud2MultiLayeredConverter::__convertSingleUnorganziedPointCloud, this, FileName);
			std::tie(FileName, TileX, TileY) = COverallConfig::getInstance()->getNextPointCloudFile();
		}
	}
	catch (std::runtime_error* e)
	{
		__waitAllActiveTaskDone();
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to execute _hiveConvertUnorganizedPointCloud2MultiLayered() due to the following reason: ", e->what()));
		_SAFE_DELETE(m_pMultiLayeredPointCloud);
	}
	catch (...)
	{
		__waitAllActiveTaskDone();
		_HIVE_OUTPUT_WARNING("Fail to execute _hiveConvertUnorganizedPointCloud2MultiLayered() due to unexpected error.");
		_SAFE_DELETE(m_pMultiLayeredPointCloud);
	}
	return m_pMultiLayeredPointCloud;
}

//*****************************************************************
//FUNCTION: 
void  CUnorganziedPointCloud2MultiLayeredConverter::__waitAllActiveTaskDone()
{
	int Counter = 0;
	while (Counter < COverallConfig::getInstance()->getAttribute<std::uint8_t>(Keywords::MAX_NUM_LOOP_WAIT_POINT_CLOUD_CONVERT_DONE).value())
	{
		int NumTaskDone = 0;
		for (auto i = 0; i < m_ActiveTaskSet.size(); i++)
		{
			if (m_ActiveTaskSet[i]._FileName.empty())
			{
				NumTaskDone++;  
				continue;
			}
			if (m_ActiveTaskSet[i]._Result.wait_for(_MILLISEC_10) == std::future_status::ready)
			{
				NumTaskDone++;
				m_ActiveTaskSet[i]._FileName.clear();
			}
		}
		if (NumTaskDone == m_ActiveTaskSet.size()) break;
	}
}

//*****************************************************************
//FUNCTION: 
CUnorganziedPointCloud2MultiLayeredConverter::SActiveTask& CUnorganziedPointCloud2MultiLayeredConverter::__updateTaskStatus()
{
	int EmptyTask = -1;

	int Counter = 0;
	while (Counter < COverallConfig::getInstance()->getAttribute<std::uint8_t>(Keywords::MAX_NUM_LOOP_WAIT_POINT_CLOUD_CONVERT_DONE).value())
	{
		for (auto i = 0; i < m_ActiveTaskSet.size(); i++)
		{
			if (!m_ActiveTaskSet[i]._FileName.empty())
			{
				if (m_ActiveTaskSet[i]._Result.wait_for(_MILLISEC_10) == std::future_status::ready)
				{
					if (!m_ActiveTaskSet[i]._Result.get()) _THROW_RUNTIME_ERROR(_FORMAT_STR1("Fail to convert the point cloud file [%1%].", m_ActiveTaskSet[i]._FileName));
					__notifyPointCloudConvertIsDone(m_ActiveTaskSet[i]._TileX, m_ActiveTaskSet[i]._TileY);
					m_ActiveTaskSet[i]._FileName.clear();
					EmptyTask = i;
				}
			}
			else
				EmptyTask = i;
		}
		if (EmptyTask != -1) return m_ActiveTaskSet[EmptyTask];
		Counter++;
	}
	_THROW_RUNTIME_ERROR("Point cloud converting tasks take too long");
}

//*****************************************************************
//FUNCTION: 
bool CUnorganziedPointCloud2MultiLayeredConverter::__convertSingleUnorganziedPointCloud(const std::string& vFileName)
{
	_ASSERTE(m_pMultiLayeredPointCloud);

	_HIVE_EARLY_RETURN(!hiveUtility::hiveFileSystem::hiveIsFileExisted(vFileName), _FORMAT_STR1("The specified point cloud file [%1%] does not exist.", vFileName), false);

	IPointCloudFileLoader* pLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudFileLoader>(COverallConfig::getInstance()->getAttribute<std::string>(Keywords::POINT_CLOUD_FILE_LOADER_SIG).value());
	_HIVE_EARLY_RETURN(!pLoader, _FORMAT_STR1("Fail to create the point cloud file loader [%1%].", COverallConfig::getInstance()->getAttribute<std::string>(Keywords::POINT_CLOUD_FILE_LOADER_SIG).value()), false);

	SUnorganizedPointCloud* pUnorganizedPointCloud = pLoader->load(vFileName);
	_ASSERTE(pUnorganizedPointCloud->isValid());

	m_pMultiLayeredPointCloud->addPointSet(pUnorganizedPointCloud);
	delete pUnorganizedPointCloud;
}

//*****************************************************************
//FUNCTION: 
void CUnorganziedPointCloud2MultiLayeredConverter::__notifyPointCloudConvertIsDone(std::uint32_t vTileX, std::uint32_t vTileY)
{
}
