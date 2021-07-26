#include "pch.h"
#include "InitialClusterCreator.h"
#include "PointCluster.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchCommon.h"

#define initResolution 512
#define DepthOffset 2

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
CPointCluster* CInitialClusterCreator::createInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel, const hiveConfig::CHiveConfig *vClusterConfig)
{

	CPointCluster* pInitialCluster = new CPointCluster;
	auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	for (auto CurrentIndex : vUserMarkedRegion)
		if (CurrentIndex < 0 || CurrentIndex >= CloudScene.getNumPoint())
			_THROW_RUNTIME_ERROR("Index is out of range");
	if(vPvMatrix == Eigen::Matrix4d{} || vClusterConfig == nullptr)
		_THROW_RUNTIME_ERROR("Empty pvmatrix or clusterconfig");
	
	const auto HardnessSet = __generateHardness4EveryPoint(vUserMarkedRegion, vPvMatrix, vHardnessFunc);
	const auto CenterIndex = __computeCenterIndex(vUserMarkedRegion, vPvMatrix);
	
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
pcl::index_t CInitialClusterCreator::__computeCenterIndex(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix) const
{
	const auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	const auto Size = vUserMarkedRegion.size();
	
	std::vector<Eigen::Vector2d> NdcCoordSet(Size);
	for (size_t i = 0; i < Size; i++)
	{
		auto Position = CloudScene.getPositionAt(i);
		Position = vPvMatrix.cast<float>() * Position;
		Position /= Position.eval().w();
		Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
		Position /= 2.0;

		NdcCoordSet[i] = { Position.x(), Position.y() };
	}

	Eigen::Vector2d Center(0.0, 0.0);
	for (auto i : NdcCoordSet)
		Center += i;
	Center /= vUserMarkedRegion.size();

	pcl::index_t CenterIndex = 0;
	double MinSquaredDistance = DBL_MAX;
	for (size_t i = 0; i < Size; i++)
	{
		auto SquaredDistance = (NdcCoordSet.at(i) - Center).squaredNorm();
		if (MinSquaredDistance > SquaredDistance)
		{
			MinSquaredDistance = SquaredDistance;
			CenterIndex = i;
		}
	}

	return CenterIndex;
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
void CInitialClusterCreator::__divideUserSpecifiedRegion(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<double>& vPointHardnessSet, float vDivideThreshold, std::vector<pcl::index_t>& voFeatureGenerationSet, std::vector<pcl::index_t>& voValidationSet)
{
	std::string OutputValidationSet = "";
	std::string OutputFeatureGenerationSet = "";

	for(size_t i = 0;i <vUserMarkedRegion.size();i++)
	{
		if (vPointHardnessSet[i] <= vDivideThreshold)
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

	//hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Remain [%1%] Validation points.\n %2%", voValidationSet.size(), OutputValidationSet));
	//hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Remain [%1%] FeatureGeneration points.\n %2%", voFeatureGenerationSet.size(), OutputFeatureGenerationSet));
}

//*****************************************************************
//FUNCTION: 
std::vector<double> CInitialClusterCreator::__generateHardness4EveryPoint(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc) const
{
	const auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	std::vector<double> HardnessSet;
	HardnessSet.resize(vUserMarkedRegion.size());
	
	for (int i = 0; i < vUserMarkedRegion.size(); i++)
	{
		Eigen::Vector4f Position = CloudScene.getPositionAt(vUserMarkedRegion[i]);
		Position = vPvMatrix.cast<float>() * Position;
		Position /= Position.eval().w();

		HardnessSet.at(i) = vHardnessFunc({ Position.x(), Position.y() });
	}

	return HardnessSet;
}
