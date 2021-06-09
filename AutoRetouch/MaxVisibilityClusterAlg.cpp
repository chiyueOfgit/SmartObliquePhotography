#include "pch.h"
#include "MaxVisibilityClusterAlg.h"
#include "PointCloudAutoRetouchScene.h"
#include <pcl/segmentation/impl/extract_clusters.hpp>

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CMaxVisibilityClusterAlg, CLASSIFIER_MaxVisibilityCluster)

//*****************************************************************
//FUNCTION:
void  CMaxVisibilityClusterAlg::runV(std::vector<std::uint64_t>& vioInputSet, EPointLabel vFinalLabel, const pcl::visualization::Camera& vCamera)
{
	if (vioInputSet.empty())
		return;
	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	auto pCloud = pScene->getPointCloudScene();
	
	const int Resolution = 64;
	//const Eigen::Vector3f ViewDir(vCamera.focal[0] - vCamera.pos[0], vCamera.focal[1] - vCamera.pos[1], vCamera.focal[2] - vCamera.pos[2]);
	const Eigen::Vector3f ViewPos(vCamera.pos[0], vCamera.pos[1], vCamera.pos[2]);
	Eigen::Matrix4d ViewMatrix;
	Eigen::Matrix4d ProjectMatrix;
	vCamera.computeViewMatrix(ViewMatrix);
	vCamera.computeProjectionMatrix(ProjectMatrix);

	std::vector<pcl::PointIndices> ClusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointSurfel> Ec;
	Ec.setClusterTolerance(0.5);
	Ec.setMinClusterSize(3);
	Ec.setMaxClusterSize(10000);
	Ec.setInputCloud(pCloud);
	Ec.setSearchMethod(pScene->getGlobalKdTree());
	Ec.setIndices(pcl::make_shared<pcl::Indices>(vioInputSet.begin(), vioInputSet.end()));
	Ec.extract(ClusterIndices);

	std::vector<std::pair<float, pcl::PointIndices*>> ClusterIndicesWithDistances;
	for (auto& Indices : ClusterIndices)
	{
		float MinDistance = FLT_MAX;
		for (auto Index : Indices.indices)
		{
			const Eigen::Vector3f Position = pCloud->at(Index).getArray3fMap();
			MinDistance = std::min(MinDistance, (Position - ViewPos).norm());
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
			
			Position = ProjectMatrix * ViewMatrix * Position;
			Position /= Position.eval().w();//»ìÏý£¿
			Position += Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
			Position *= Resolution / 1.0;

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
	vioInputSet= std::vector<std::uint64_t>(pMaxValidCluster->indices.begin(), pMaxValidCluster->indices.end());;
}