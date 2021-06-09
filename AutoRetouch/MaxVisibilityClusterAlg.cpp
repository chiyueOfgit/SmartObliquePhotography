#include "pch.h"
#include "MaxVisibilityClusterAlg.h"
#include "PointCloudAutoRetouchScene.h"
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include "SpatialClusterConfig.h"
#include <common/ConfigInterface.h>
#include <common/FileSystem.h>
#include <common/CommonMicro.h>

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CMaxVisibilityClusterAlg, CLASSIFIER_MaxVisibilityCluster)

//*****************************************************************
//FUNCTION:
void  CMaxVisibilityClusterAlg::runV(std::vector<std::uint64_t>& vioInputSet, EPointLabel vFinalLabel, const pcl::visualization::Camera& vCamera)
{
	hiveConfig::EParseResult IsParsedDisplayConfig = hiveConfig::hiveParseConfig("SpatialClusterConfig.xml", hiveConfig::EConfigType::XML, CDisplayConfig::getInstance());
	if (IsParsedDisplayConfig != hiveConfig::EParseResult::SUCCEED)
	{
		std::cout << "Failed to parse config file." << std::endl;
		system("pause");
		return;
	}

	if (vioInputSet.empty())
		return;
	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	auto pCloud = pScene->getPointCloudScene();
	pcl::search::KdTree<pcl::PointSurfel>::Ptr pTree(new pcl::search::KdTree<pcl::PointSurfel>);
	
	const int  Resolution = *CDisplayConfig::getInstance()->getAttribute<int>(KEY_WORDS::RESOLUTION);
	const Eigen::Vector3f ViewDir(vCamera.focal[0] - vCamera.pos[0], vCamera.focal[1] - vCamera.pos[1], vCamera.focal[2] - vCamera.pos[2]);
	const Eigen::Vector3f ViewPos(vCamera.pos[0], vCamera.pos[1], vCamera.pos[2]);
	Eigen::Matrix4d ViewMatrix = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d ProjectMatrix = Eigen::Matrix4d::Identity();
	vCamera.computeViewMatrix(ViewMatrix);
	vCamera.computeProjectionMatrix(ProjectMatrix);

	pcl::PointCloud<pcl::PointSurfel>::Ptr pCloudWithIndex(new pcl::PointCloud<pcl::PointSurfel>);
	for (auto Index : vioInputSet)
	{
		pcl::PointSurfel& Point = pCloud->points[Index];
		Point.curvature = Index;
		pCloudWithIndex->push_back(Point);
	}

	pTree->setInputCloud(pCloudWithIndex);
	std::vector<pcl::PointIndices> ClusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointSurfel> Ec;
	Ec.setClusterTolerance(*CDisplayConfig::getInstance()->getAttribute<double>(KEY_WORDS::CLUSTERTOLERANCE));
	Ec.setMinClusterSize(*CDisplayConfig::getInstance()->getAttribute<int>(KEY_WORDS::MINCLUSTERSIZE));
	Ec.setMaxClusterSize(*CDisplayConfig::getInstance()->getAttribute<int>(KEY_WORDS::MAXCLUSTERSIZE));
	Ec.setSearchMethod(pTree);
	Ec.setInputCloud(pCloudWithIndex);
	Ec.extract(ClusterIndices);

	std::map<float, pcl::PointCloud<pcl::PointSurfel>::Ptr> CloudMap;
	for (auto& [Header, Indices] : ClusterIndices)
	{
		float Min = FLT_MAX;
		pcl::PointCloud<pcl::PointSurfel>::Ptr CloudCluster(new pcl::PointCloud<pcl::PointSurfel>);
		for (auto Index : Indices)
		{
			CloudCluster->points.push_back(pCloudWithIndex->points[Index]);
			Eigen::Vector3f TempPoint{ pCloudWithIndex->points[Index].x, pCloudWithIndex->points[Index].y, pCloudWithIndex->points[Index].z };
			Eigen::Vector3f TempNormal{ pCloudWithIndex->points[Index].normal_x, pCloudWithIndex->points[Index].normal_y, pCloudWithIndex->points[Index].normal_z };
			Min = Min < (TempPoint - ViewPos).norm() ? Min : (TempPoint - ViewPos).norm();
		}
		CloudMap.insert(std::pair<float, pcl::PointCloud<pcl::PointSurfel>::Ptr>(Min, CloudCluster));
	}

	std::vector<std::vector<bool>> Flag(Resolution, std::vector<bool>(Resolution, true));
	std::vector<int> ValidSet{};
	for (auto& CloudPair : CloudMap)
	{
		int Num = 0;
		for (auto& Point : CloudPair.second->points)
		{
			Eigen::Vector4d TempPoint(Point.x, Point.y, Point.z, 1.0);
			TempPoint = ProjectMatrix * ViewMatrix * TempPoint;
			Eigen::Vector2i Coord{ (TempPoint[0] / TempPoint[3] + 1) * Resolution / 2, (TempPoint[1] / TempPoint[3] + 1) * Resolution / 2 };
			if (Coord[0] > 0 && Coord[0] < Resolution && Coord[1] > 0 && Coord[1] < Resolution && Flag[Coord[0]][Coord[1]])
			{
				Flag[Coord[0]][Coord[1]] = false;
				Num++;
			}
		}
		ValidSet.emplace_back(Num);
	}
	auto offset = distance(ValidSet.begin(), max_element(ValidSet.begin(), ValidSet.end()));
	auto MaxPosition = CloudMap.begin();
	advance(MaxPosition, offset);

	std::vector<std::size_t> OutIndex;
	if (!CloudMap.size())return;
	pcl::PointCloud<pcl::PointSurfel>::Ptr FinalCloud = MaxPosition->second;
	for (auto& Point : FinalCloud->points)
	{
		m_pLocalLabelSet->changePointLabel(Point.curvature, vFinalLabel);
		OutIndex.emplace_back(Point.curvature);
	}
	vioInputSet.swap(OutIndex);
}