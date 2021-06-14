#include "pch.h"
#include "MaxVisibilityClusterAlg.h"
#include "PointCloudAutoRetouchScene.h"
#include <common/FileSystem.h>

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CMaxVisibilityClusterAlg, CLASSIFIER_MaxVisibilityCluster)

//*****************************************************************
//FUNCTION:
void  CMaxVisibilityClusterAlg::runV(const pcl::IndicesPtr& vioPointSet, EPointLabel vFinalLabel, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix)
{
	if (hiveConfig::hiveParseConfig("AutoRetouchConfig.xml", hiveConfig::EConfigType::XML, CAutoRetouchConfig::getInstance()) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", "AutoRetouchConfig.xml"));
		return;
	}

	if (vioPointSet == nullptr || vioPointSet->empty())
		return;
	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	auto pCloud = pScene->getPointCloudScene();
	pcl::search::KdTree<pcl::PointSurfel>::Ptr pTree(new pcl::search::KdTree<pcl::PointSurfel>);
	pTree->setInputCloud(pCloud);

	const int  Resolution = *CAutoRetouchConfig::getInstance()->getAttribute<int>(KEY_WORDS::RESOLUTION);

	std::vector<pcl::PointIndices> ClusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointSurfel> Ec;
	Ec.setClusterTolerance(*CAutoRetouchConfig::getInstance()->getAttribute<double>(KEY_WORDS::CLUSTERTOLERANCE));
	Ec.setMinClusterSize(*CAutoRetouchConfig::getInstance()->getAttribute<int>(KEY_WORDS::MINCLUSTERSIZE));
	Ec.setMaxClusterSize(*CAutoRetouchConfig::getInstance()->getAttribute<int>(KEY_WORDS::MAXCLUSTERSIZE));
	Ec.setInputCloud(pCloud);
	Ec.setSearchMethod(pTree);
	Ec.setIndices(vioPointSet);
	Ec.extract(ClusterIndices);
	if (ClusterIndices.empty())
		return;

	std::vector<std::pair<float, pcl::PointIndices*>> ClusterIndicesWithDistances;
	for (auto& Indices : ClusterIndices)
	{
		float MinDistance = FLT_MAX;
		for (auto Index : Indices.indices)
		{
			const Eigen::Vector3f Position = pCloud->at(Index).getArray3fMap();
			MinDistance = std::min(MinDistance, (Position - vCameraPos).norm());
		}
		ClusterIndicesWithDistances.emplace_back(MinDistance, &Indices);
	}
	std::sort(ClusterIndicesWithDistances.begin(), ClusterIndicesWithDistances.end());

	std::vector Flag(Resolution, std::vector(Resolution, true));
	int MaxValidCount = 0;
	pcl::PointIndices* pMaxValidCluster = ClusterIndicesWithDistances.front().second;
	for (auto& [MinDistance, pClusterIndices] : ClusterIndicesWithDistances)
	{
		int ValidCount = 0;
		for (auto Index : pClusterIndices->indices)
		{
			Eigen::Vector4d Position = pCloud->at(Index).getVector4fMap().cast<double>();
			//w?
			Position.w() = 1.0;
			
			Position = vPvMatrix * Position;
			Position /= Position.eval().w();//»ìÏý£¿
			Position += Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
			Position *= Resolution / 2.0;

			Eigen::Vector2i Coord{ Position.x(), Position.y() };
			if (Coord[0] > 0 && Coord[0] < Resolution && Coord[1] > 0 && Coord[1] < Resolution && Flag[Coord[0]][Coord[1]])
			{
				Flag[Coord[0]][Coord[1]] = false;
				++ValidCount;
			}
		}
		if (ValidCount > MaxValidCount)
		{
			MaxValidCount = ValidCount;
			pMaxValidCluster = pClusterIndices;
		}
	}

	
	for (auto Index : pMaxValidCluster->indices)
		m_pLocalLabelSet->changePointLabel(Index, vFinalLabel);

	vioPointSet->swap(pMaxValidCluster->indices);
}