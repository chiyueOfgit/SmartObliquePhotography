#include "pch.h"
#include "InitialClusterCreator.h"
#include "PointCluster.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchCommon.h"
#include "ScreenSpaceOperation.h"

#define initResolution 512
#define DepthOffset 2

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
CPointCluster* CInitialClusterCreator::createInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, float vHardness, EPointLabel vTargetLabel, const hiveConfig::CHiveConfig *vClusterConfig)
{
	CPointCluster* pInitialCluster = new CPointCluster;
	auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	const auto DistanceSet = __computeDistanceSetFromCenter(vUserMarkedRegion, vPvMatrix);
	const auto HardnessSet = __generateHardness4EveryPoint(DistanceSet, vHardness);
	const std::uint32_t CenterIndex = std::min_element(DistanceSet.begin(), DistanceSet.end()) - DistanceSet.begin();
	
	std::optional<float> DivideThreshold = vClusterConfig->getAttribute<float>("HARDNESS_THRESHOLD");
	if (!DivideThreshold.has_value())
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("The threshold for dividing point set is not defined. The default value [%1%] is employed.", m_DefaultPointSetDivideThreshold));
		DivideThreshold = m_DefaultPointSetDivideThreshold;
	}

	std::vector<pcl::index_t> FeatureGenerationSet, ValidationSet;
	__divideUserSpecifiedRegion(vUserMarkedRegion, HardnessSet, DivideThreshold.value(), FeatureGenerationSet, ValidationSet);

	pInitialCluster->init(vClusterConfig, CenterIndex, vTargetLabel, FeatureGenerationSet, ValidationSet, CPointCloudRetouchManager::getInstance()->addAndGetTimestamp());

	return pInitialCluster;
}

//*****************************************************************
//FUNCTION: 
std::vector<float> CInitialClusterCreator::__computeDistanceSetFromCenter(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix)
{
	const auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	const auto Size = vUserMarkedRegion.size();
	
	std::vector<Eigen::Vector2f> NdcCoordSet(Size);
	for (size_t i = 0; i < Size; i++)
	{
		auto Position = CloudScene.getPositionAt(i);
		Position = vPvMatrix.cast<float>() * Position;
		Position /= Position.eval().w();
		Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
		Position /= 2.0;

		NdcCoordSet[i] = { Position.x(), Position.y() };
	}

	Eigen::Vector2f Center(0.0f, 0.0f);
	for (auto i : NdcCoordSet)
		Center += i;
	Center /= vUserMarkedRegion.size();

	std::vector<float> DistanceSet(Size);
	for (size_t i = 0; i < Size; i++)
		DistanceSet[i] = (NdcCoordSet[i] - Center).norm();
	
	return DistanceSet;
}

void OutputMessage(pcl::index_t vUserMarkedRegionPoint, std::string& vioOutputString)
{
	std::string OutputOnePointMessage = "";
	auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	Eigen::Vector4f Position = CloudScene.getPositionAt(vUserMarkedRegionPoint);
	OutputOnePointMessage = " id: " + std::to_string(vUserMarkedRegionPoint) + "\t\t(" + std::to_string(Position[0]) + ", " + std::to_string(Position[1]) + ", " + std::to_string(Position[2]) + ")\n ";
	vioOutputString += OutputOnePointMessage;
}

//*****************************************************************
//FUNCTION: 
void CInitialClusterCreator::__divideUserSpecifiedRegion(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<float>& vPointHardnessSet, float vDivideThreshold, std::vector<pcl::index_t>& voFeatureGenerationSet, std::vector<pcl::index_t>& voValidationSet)
{
	std::string OutputValidationSet = "";
	std::string OutputFeatureGenerationSet = "";

	for(size_t i = 0;i <vUserMarkedRegion.size();i++)
	{
		if (vPointHardnessSet[i] < vDivideThreshold && vPointHardnessSet[i] != 0.0)
		{
			voValidationSet.push_back(vUserMarkedRegion[i]);
			OutputMessage(vUserMarkedRegion[i], OutputValidationSet);
		}
		else if (vPointHardnessSet[i] > vDivideThreshold)
		{
			voFeatureGenerationSet.push_back(vUserMarkedRegion[i]);
			OutputMessage(vUserMarkedRegion[i], OutputFeatureGenerationSet);
		}
	}

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Remain [%1%] Validation points.\n %2%", voValidationSet.size(), OutputValidationSet));
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Remain [%1%] FeatureGeneration points.\n %2%", voFeatureGenerationSet.size(), OutputFeatureGenerationSet));
}

//*****************************************************************
//FUNCTION: 
std::vector<float> CInitialClusterCreator::__generateHardness4EveryPoint(const std::vector<float>& vDistanceSetFromCenter, float vHardness)
{
	const auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	const auto Size = vDistanceSetFromCenter.size();
	
	const auto MaxDistance = *std::max_element(vDistanceSetFromCenter.begin(), vDistanceSetFromCenter.end());
	std::vector<float> HardnessSet;
	HardnessSet.reserve(Size);
	for (auto i : vDistanceSetFromCenter)
	{
		if (i <= vHardness * MaxDistance)
			HardnessSet.push_back(1.0f);
		else
			HardnessSet.push_back(0.0f);
	}

	return HardnessSet;
}
