#include "pch.h"
#include "PointCloudRetouchManager.h"

#include "BoundaryDetector.h"
#include "PointCluster.h"
#include "NeighborhoodBuilder.h"
#include "OutlierDetector.h"

#include "ColorFeature.h"
#include "PointClusterExpanderMultithread.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::init(const std::vector<PointCloud_t::Ptr>& vTileSet, const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(!vTileSet.empty() && vConfig);

	bool Status = __reset();
	_ASSERTE(Status);

	m_Scene.init(vTileSet);
	m_PointLabelSet.init(m_Scene.getNumPoint());
	m_pConfig = vConfig;

	if (_IS_STR_IDENTICAL(vConfig->getSubconfigAt(0)->getSubconfigType(), std::string("POINT_CLOUD_RETOUCN_CONFIG")))
	{
		_ASSERTE(vConfig->getNumSubconfig());
		if (_IS_STR_IDENTICAL(vConfig->getSubconfigAt(0)->getName(), std::string("Retouch")))
			vConfig = vConfig->getSubconfigAt(0);
	}

	auto num = vConfig->getNumSubconfig();

	for (int i = 0; i < num; i++)
	{
		const hiveConfig::CHiveConfig* pConfig = vConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("TASK")))
		{
			if (_IS_STR_IDENTICAL(pConfig->getName(), std::string("LitterMarker")))
			{
				//for precompute
				for (auto k = 0; k < pConfig->getNumSubconfig(); k++)
				{
					const hiveConfig::CHiveConfig* pClusterConfig = pConfig->getSubconfigAt(k);
					if (_IS_STR_IDENTICAL(pClusterConfig->getSubconfigType(), std::string("CLUSTER")))
					{
						m_pPrecomputeManager = new CPrecomputeManager;
						m_pPrecomputeManager->init(pClusterConfig);
					}
				}
				m_LitterMarker.init(pConfig);
			}
			else if (_IS_STR_IDENTICAL(pConfig->getName(), std::string("BackgroundMarker")))
			{
				m_BackgroundMarker.init(pConfig);	
			}
			else if (_IS_STR_IDENTICAL(pConfig->getName(), std::string("AutoMarker")))
			{
				m_pAutoMarkerConfig = pConfig;
			}
			else
			{
				_HIVE_EARLY_RETURN(true, _FORMAT_STR1("Unexpeced subconfiguration [%1%].", pConfig->getName()), false);
			}
			continue;
		}
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("NEIGHBOR_BUILDER")))
		{
			std::optional<std::string> NeighborhoodBuilderSig = pConfig->getAttribute<std::string>("SIG");
			_ASSERTE(NeighborhoodBuilderSig.has_value());
			m_pNeighborhoodBuilder = hiveDesignPattern::hiveCreateProduct<INeighborhoodBuilder>(NeighborhoodBuilderSig.value(), pConfig, vTileSet, &m_PointLabelSet);
			_HIVE_EARLY_RETURN(!m_pNeighborhoodBuilder, _FORMAT_STR1("Fail to initialize retouch due to the failure of creating neighborhood builder [%1%].", NeighborhoodBuilderSig.value()), false);
			continue;
		}
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("OUTLIER")))
		{
			if (_IS_STR_IDENTICAL(pConfig->getName(), std::string("Outlier")))
			{
				m_pOutlierConfig = pConfig;
			}
			continue;
		}
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("HOLE_REPAIRER")))
		{
			if (_IS_STR_IDENTICAL(pConfig->getName(), std::string("HoleRepairer")))
			{
				m_HoleRepairer.init(pConfig);
			}
			continue;
		}
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Unknown subconfiguration type [%1%].", pConfig->getSubconfigType()));
	}
	recordCurrentStatus();
	return true;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::__reset()
{
	try
	{
		m_pConfig = nullptr;
		m_pOutlierConfig = nullptr;

		m_Timestamp = 0;

		m_LitterMarker.reset();
		m_BackgroundMarker.reset();

		m_Scene.reset();
		m_PointClusterSet.reset();

		if (m_pNeighborhoodBuilder)
		{
			m_pNeighborhoodBuilder->reset();
			m_pNeighborhoodBuilder = nullptr;
		}
		m_PointLabelSet.clear();
		m_StatusQueue.clear();
		return true;
	}
	catch (std::runtime_error&)
	{

	}
	catch (...)
	{

	}
	return false;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::clearMark()
{
	m_PointLabelSet.reset();
	if (m_pNeighborhoodBuilder)
		m_pNeighborhoodBuilder->reset();
	m_PointClusterSet.reset();
}

//*****************************************************************
//FUNCTION: 
CPointCluster* CPointCloudRetouchManager::__generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel)
{
	_ASSERTE(m_pConfig);
	_ASSERTE((vTargetLabel == EPointLabel::KEPT) || (vTargetLabel == EPointLabel::UNWANTED));

	const hiveConfig::CHiveConfig* pClusterConfig = (vTargetLabel == EPointLabel::KEPT) ? m_BackgroundMarker.getClusterConfig() : m_LitterMarker.getClusterConfig();
	_ASSERTE(pClusterConfig);

	return m_InitialClusterCreator.createInitialCluster(vUserMarkedRegion, vPvMatrix, vHardnessFunc, vTargetLabel, pClusterConfig);
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::dumpPointLabel(std::vector<std::size_t>& voPointLabel) const
{
	auto NumPoints = m_Scene.getNumPoint();
	if (NumPoints > 0)
	{
		voPointLabel.clear();

		for (int i = 0; i < NumPoints; i++)
		{
			auto Label = m_PointLabelSet.getLabelAt(i);
			voPointLabel.push_back(static_cast<std::size_t>(Label));
		}
		
		return true;
	}
	else
		return false;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::dumpTileLabel(std::size_t vTile, std::vector<std::size_t>& voTileLabel)
{
	if (vTile <= m_Scene.getNumTile())
	{
		voTileLabel.clear();
		auto Start = m_Scene.getTileOffset(vTile);
		auto End = m_Scene.getTileNumPoints(vTile) + Start;
		for (auto i = Start; i < End; i++)
			voTileLabel.push_back(static_cast<std::size_t>(m_PointLabelSet.getLabelAt(i)));
		return true;
	}
	else
		return false;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::dumpPointLabelAt(std::size_t& voPointLabel, std::uint32_t vIndex) const
{
	auto NumPoints = m_Scene.getNumPoint();
	if (vIndex < NumPoints)
	{
		auto Label = m_PointLabelSet.getLabelAt(vIndex);
		voPointLabel = static_cast<std::size_t>(Label);
		
		return true;
	}
	else
		return false;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::executePreprocessor(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vSignedDistanceFunc, const Eigen::Vector3d& vViewPos)
{
	//TODO: 完善
	try
	{
		m_Preprocessor.cullBySdf(vioPointSet, vPvMatrix, vSignedDistanceFunc);
		m_Preprocessor.cullByDepth(vioPointSet, vPvMatrix, vViewPos);
		return true;
	}
	catch (std::runtime_error&)
	{

	}
	catch (...)
	{

	}
	return false;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::dumpColorFeatureMainColors(std::vector<Eigen::Vector3i>& vMainColors) const
{
	auto pCluster = m_PointClusterSet.getLastCluster();
	const CColorFeature* pColorFeature = nullptr;
	if (pCluster)
	{
		pColorFeature = dynamic_cast<const CColorFeature*>(pCluster->getFeatureBySig(KEYWORD::COLOR_FEATURE));
		if (pColorFeature)
		{
			vMainColors = pColorFeature->getMainBaseColors();
			return true;
		}
	}

	return false;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::dumpColorFeatureNearestPoints(std::vector<pcl::index_t>& vNearestPoints) const
{
	auto pCluster = m_PointClusterSet.getLastCluster();
	const CColorFeature* pColorFeature = nullptr;
	if (pCluster)
	{
		pColorFeature = dynamic_cast<const CColorFeature*>(pCluster->getFeatureBySig(KEYWORD::COLOR_FEATURE));
		if (pColorFeature)
		{
			vNearestPoints = pColorFeature->getMainColorsNearestPoints();
			return true;
		}
	}

	return false;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel)
{
	_ASSERTE((vTargetLabel == EPointLabel::UNWANTED) || (vTargetLabel == EPointLabel::KEPT));

	try
	{
		CPointCluster* pInitCluster = __generateInitialCluster(vUserMarkedRegion, vPvMatrix, vHardnessFunc, vTargetLabel);
		_ASSERTE(pInitCluster);
		m_PointClusterSet.addCluster(pInitCluster);
		m_pNeighborhoodBuilder->reset();
		//m_PointLabelSet.tagCoreRegion4Cluster(pInitCluster->getCoreRegion(), vTargetLabel, pInitCluster->getClusterIndex());

		if (vTargetLabel == EPointLabel::UNWANTED)
		{
			return m_LitterMarker.execute(pInitCluster);
		}
		else
		{
			return m_BackgroundMarker.execute(pInitCluster);
		}
	}
	catch (std::runtime_error& e)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to execute marker because of the following error: [%1%].", e.what()));
	}
	catch (...)
	{
		_HIVE_OUTPUT_WARNING("Fail to execute marker due to unexpected error");
	}
//TODO: 当执行过程中出现异常时，需要将状态重置为函数执行之前（需要测试用例加以验证）
	return false;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::switchLabel(EPointLabel vTo, EPointLabel vFrom)
{
	auto NumPoints = m_Scene.getNumPoint();
	_ASSERTE(NumPoints > 0);

	for (int i = 0; i < NumPoints; i++)
	{
		if (m_PointLabelSet.getLabelAt(i) == vFrom)
			m_PointLabelSet.tagPointLabel(i, vTo, m_PointLabelSet.getClusterIndexAt(i), m_PointLabelSet.getClusterBelongingProbabilityAt(i));
	}
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::setLabel(const std::vector<pcl::index_t>& vPoints, EPointLabel vTarget)
{
	auto NumPoints = m_Scene.getNumPoint();
	_ASSERTE(NumPoints > 0);

	for (auto Index : vPoints)
	{
		auto Label = m_PointLabelSet.getLabelAt(Index);
		if (Label == EPointLabel::KEPT || Label == EPointLabel::UNWANTED)
			m_PointLabelSet.tagPointLabel(Index, vTarget, 0, 0);
	}
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CPointCloudRetouchManager::buildNeighborhood(pcl::index_t vSeed, std::string& vType, float vPara)
{
	//发生NRVO
	return m_pNeighborhoodBuilder->buildNeighborhood(vSeed, vType, vPara);
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CPointCloudRetouchManager::buildNeighborhood(pcl::index_t vSeed)
{
	return m_pNeighborhoodBuilder->buildNeighborhood(vSeed);
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::dumpIndicesByLabel(std::vector<pcl::index_t>& vioIndices, EPointLabel vLabel)
{
	for (size_t i = 0; i < m_PointLabelSet.getSize(); i++)
		if (m_PointLabelSet.getLabelAt(i) == vLabel)
			vioIndices.push_back(i);
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::recoverMarkedPoints2Undetermined(EPointLabel vLabel)
{
	_ASSERTE(vLabel == EPointLabel::UNWANTED || vLabel == EPointLabel::KEPT);
	std::vector<int> Indices;
	dumpIndicesByLabel(Indices, vLabel);
	for (auto Index : Indices)
		tagPointLabel(Index, EPointLabel::UNDETERMINED, 0, 0);
	m_pNeighborhoodBuilder->reset();
	m_PointClusterSet.removeClustersByLabel(vLabel);
	recordCurrentStatus();
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::executeOutlierDetector(EPointLabel vTargetLabel)
{
	std::vector<pcl::index_t> Indices;
	dumpIndicesByLabel(Indices, EPointLabel::UNDETERMINED);
	dumpIndicesByLabel(Indices, EPointLabel::KEPT);
	auto pOutlierDetector = dynamic_cast<COutlierDetector*>(hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>("OUTLIER_DETECTOR"));
	return pOutlierDetector->execute<COutlierDetector>(Indices, vTargetLabel, m_pOutlierConfig->getAttribute<float>("SEARCH_RADIUS").value(), m_pOutlierConfig->getAttribute<int>("MIN_NEIGHBORS_IN_RADIUS").value(), m_pOutlierConfig->getAttribute<bool>("POINT_FILTER_CONDITION").value());
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::undo()
{
	if (m_StatusQueue.size() > 1)
		m_StatusQueue.pop_back();
	else
		return false;

	m_PointClusterSet.removeLastCluster();

	auto LastStatus = m_StatusQueue.back();
	m_PointLabelSet = LastStatus.first;
	m_Timestamp = LastStatus.second;
	return true;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::recordCurrentStatus()
{
	m_StatusQueue.emplace_back(m_PointLabelSet, m_Timestamp);
	if (m_StatusQueue.size() > 10)
		m_StatusQueue.pop_front();
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::executeHoleRepairerSetRegion(const std::vector<pcl::index_t>& vHoleRegion)
{
	m_HoleRepairer.setHoleRegion(vHoleRegion);
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::executeHoleRepairerSetInput(const std::vector<pcl::index_t>& vInput)
{
	m_HoleRepairer.setInput(vInput);
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::executeHoleRepairer(std::vector<pcl::PointXYZRGBNormal>& voNewPoints)
{
	m_HoleRepairer.repairHole(voNewPoints);
}

//*****************************************************************
//FUNCTION: 
std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f> CPointCloudRetouchManager::calcOBBByIndices(const std::vector<pcl::index_t>& vIndices)
{
	std::vector<Eigen::Vector3f> PosSet;
	for(auto Index:vIndices)
	{
		auto Pos = m_Scene.getPositionAt(Index);
		PosSet.push_back({ Pos.x(),Pos.y(),Pos.z() });
	}
	return calcOBB(PosSet);
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::executeAutoMarker()
{
	auto AutoMarkerResolution = m_pAutoMarkerConfig->getAttribute<std::tuple<int, int>>("AUTOMARKER_RESOLUTION").value();
	Eigen::Vector2i Resolution = { std::get<0>(AutoMarkerResolution), std::get<1>(AutoMarkerResolution) };
	std::vector<pcl::index_t> OutPutIndices;
	std::vector<std::vector<pcl::index_t>> EdgeIndices;
	auto pExtractor = hiveDesignPattern::hiveCreateProduct<CGroundObjectExtractor>(KEYWORD::GROUND_OBJECT_EXTRACTOR);
	if (!pExtractor)
		std::cerr << "create Extractor error." << std::endl;
	pExtractor->execute<CGroundObjectExtractor>(OutPutIndices, EdgeIndices, Resolution);

	for (auto Index : OutPutIndices)
		tagPointLabel(Index, EPointLabel::KEPT, 0, 1.0);
	
	CPointCluster* pInitialCluster = new CPointCluster;
    const hiveConfig::CHiveConfig* pClusterConfig = m_BackgroundMarker.getClusterConfig();
	CPointClusterExpanderMultithread* pPointClusterExpander = dynamic_cast<CPointClusterExpanderMultithread*>(hiveDesignPattern::hiveCreateProduct<IPointClassifier>("CLUSTER_EXPANDER_MULTITHREAD"));
	for(auto& EdgeSet: EdgeIndices)
	{
		if(EdgeSet.size())
		{
			std::vector<pcl::index_t> ValidationSet;
			std::vector<pcl::index_t>::iterator Iter = EdgeSet.begin();
			ValidationSet.push_back(*Iter);
			EdgeSet.erase(Iter);
	
			pInitialCluster->init(pClusterConfig, 0, EPointLabel::KEPT, EdgeSet, ValidationSet, addAndGetTimestamp());
			pPointClusterExpander->runV(pInitialCluster);
		}
	}
	switchLabel(EPointLabel::UNWANTED, EPointLabel::UNDETERMINED);
	recoverMarkedPoints2Undetermined(EPointLabel::KEPT);
}