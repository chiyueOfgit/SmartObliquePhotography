#include "pch.h"
#include "InitialClusterCreator.h"
#include "PointCluster.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchCommon.h"

#define initResolution 128
#define DepthOffset 0.4

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
CPointCluster* CInitialClusterCreator::createInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, float vHardness, float vRadius,  EPointLabel vLabel, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, const hiveConfig::CHiveConfig *vClusterConfig)
{
	CPointCluster* pInitialCluster = new CPointCluster;

	std::vector<float> PointHardnessSet(vUserMarkedRegion.size(), 0.0);
	__generateHardness4EveryPoint(vUserMarkedRegion, vHardness, vRadius, vCenter, vPvMatrix, vWindowSize, PointHardnessSet);

	pcl::index_t ClusterCenter = __computeClusterCenter(vUserMarkedRegion, PointHardnessSet, vCenter, vPvMatrix, vWindowSize);

	std::optional<float> DivideThreshold = vClusterConfig->getAttribute<float>("HARDNESS_THRESHOLD");
	if (!DivideThreshold.has_value())
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("The threshold for dividing point set is not defined. The default value [%1%] is employed.", m_DefaultPointSetDivideThreshold));
		DivideThreshold = m_DefaultPointSetDivideThreshold;
	}

	std::vector<pcl::index_t> FeatureGenerationSet, ValidationSet;
	__divideUserSpecifiedRegion(vUserMarkedRegion, PointHardnessSet, DivideThreshold.value(), FeatureGenerationSet, ValidationSet);

	pInitialCluster->init(vClusterConfig, ClusterCenter, vLabel, FeatureGenerationSet, ValidationSet, CPointCloudRetouchManager::getInstance()->addAndGetTimestamp());

	return pInitialCluster;
}

//*****************************************************************
//FUNCTION: 
void CInitialClusterCreator::__divideUserSpecifiedRegion(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<float> vPointHardnessSet, float vDivideThreshold, std::vector<pcl::index_t>& voFeatureGenerationSet, std::vector<pcl::index_t>& voValidationSet)
{
	for(size_t i = 0;i <vUserMarkedRegion.size();i++)
	{
		if (vPointHardnessSet[i] < vDivideThreshold && vPointHardnessSet[i] != 0.0)
			voValidationSet.push_back(vUserMarkedRegion[i]);
		else if(vPointHardnessSet[i] > vDivideThreshold)
			voFeatureGenerationSet.push_back(vUserMarkedRegion[i]);
	}
}

//*****************************************************************
//FUNCTION: 
pcl::index_t CInitialClusterCreator::__computeClusterCenter(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<float> vPointHardnessSet, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize)
{
	pcl::index_t CenterIndex;
	float MinDistance = FLT_MAX;
	
	for(size_t i = 0; i < vUserMarkedRegion.size(); i++)
	{
		if (vPointHardnessSet[i] > 0)
		{
			auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
			Eigen::Vector4f Position = CloudScene.getPositionAt(vUserMarkedRegion[i]);

			Position = vPvMatrix.cast<float>() * Position;
			Position /= Position.eval().w();
			Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
			Position /= 2.0;
			Eigen::Vector2f Coord{ Position.x() * vWindowSize.first, Position.y() * vWindowSize.second };
			if ((Coord - vCenter).norm() < MinDistance)
			{
				MinDistance = (Coord - vCenter).norm();
				CenterIndex = vUserMarkedRegion[i];
			}
		}
	}
	return CenterIndex;
}

//*****************************************************************
//FUNCTION: 
void CInitialClusterCreator::__generateHardness4EveryPoint(const std::vector<pcl::index_t>& vUserMarkedRegion, float vHardness, float vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, std::vector<float>& voPointHardnessSet)
{
	_ASSERTE(vRadius);
	
	int Resolution = initResolution;
	float MinDepth = FLT_MAX;
	std::vector<Eigen::Vector4f> MarkedRegionScreenCoord;
	std::vector<std::vector<std::pair<float,int>>> Raster(Resolution, std::vector(Resolution, std::pair(FLT_MAX,-1)));
	
	for(size_t i = 0;i < vUserMarkedRegion.size();i++)
	{
	    auto CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
		Eigen::Vector4f Position = CloudScene.getPositionAt(vUserMarkedRegion[i]);
		
		Position = vPvMatrix.cast<float>() * Position;
		Position /= Position.eval().w();
		Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
		Position /= 2.0;
		MarkedRegionScreenCoord.push_back(Position);
		Position *= Resolution;
		Eigen::Vector3f Coord{ Position.x(), Position.y(), Position.z() };
		
		if (Coord[0] > 0 && Coord[0] < Resolution && Coord[1] > 0 && Coord[1] < Resolution && Coord[2] < Raster[Coord[0]][Coord[1]].first)
		{
			Raster[Coord[0]][Coord[1]].first = Coord[2];
			Raster[Coord[0]][Coord[1]].second = i;
			if (Coord[2] < MinDepth)
				MinDepth = Coord[2];
		}
	}

	for(auto& Lines: Raster)
		for(auto& Pair: Lines)
		{
			if(Pair.second > -1 && (Pair.first - MinDepth) < DepthOffset)
			{
				Eigen::Vector2f CoordXY = { MarkedRegionScreenCoord[Pair.second][0] * vWindowSize.first, MarkedRegionScreenCoord[Pair.second][1] * vWindowSize.second };
				float Rate = (CoordXY - vCenter).norm() / vRadius;
				if (Rate < vHardness)
					voPointHardnessSet[Pair.second] = 1;
				else if(Rate > vHardness && Rate < 1)
					voPointHardnessSet[Pair.second] = NormalDistribution<float>(2 * (Rate - vHardness) /(1 - vHardness));
			}
		}
}
