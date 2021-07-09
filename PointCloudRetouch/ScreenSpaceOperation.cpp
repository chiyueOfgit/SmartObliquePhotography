#include "pch.h"
#include "ScreenSpaceOperation.h"
#include "PointCloudRetouchManager.h"
#include <omp.h>

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CScreenSpaceOperation::cullByDepth(std::vector<pcl::index_t>& vioPointIndices, const hiveConfig::CHiveConfig* vClusterConfig)
{
	auto& Scene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	static int AreaWidth = m_RightDown.x() - m_LeftUp.x() + 1;
	static int AreaHeight = m_RightDown.y() - m_LeftUp.y() + 1;

	std::vector<float> PointsDepth(AreaWidth * AreaHeight, FLT_MAX);
	std::set<pcl::index_t> ResultPoints;

	Eigen::Matrix4d PVInverse = m_PvMatrix.inverse();

	//tile projection
	const std::size_t NumPartitionX = 0.5 * AreaWidth, NumPartitionY = 0.5 * AreaHeight;
	float TileDeltaX = (float)AreaWidth / NumPartitionX;
	float TileDeltaY = (float)AreaHeight / NumPartitionY;

	std::vector<std::vector<pcl::index_t>> PointTile(NumPartitionX * NumPartitionY);

	std::mutex Mutex;

#pragma omp parallel for
	for (int i = 0; i < vioPointIndices.size(); i++)
	{
		auto Index = vioPointIndices[i];
		Eigen::Vector4f Position = Scene.getPositionAt(Index);

		Position = m_PvMatrix * Position;
		Position /= Position.eval().w();
		Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
		Position /= 2.0;
		Position.x() *= m_WindowSize.x();
		Position.y() *= m_WindowSize.y();

		int TileIndexX = (Position.x() - m_LeftUp.x()) / TileDeltaX;
		int TileIndexY = (Position.y() - m_LeftUp.y()) / TileDeltaY;

		_ASSERTE(TileIndexX >= 0 && TileIndexY >= 0);

		Mutex.lock();
		PointTile[TileIndexX + TileIndexY * NumPartitionX].push_back(Index);
		Mutex.unlock();
	}
	std::mutex Mutex;

#pragma omp parallel for
	for (int Y = m_LeftUp.y(); Y <= m_RightDown.y(); Y++)
	{
		for (int X = m_LeftUp.x(); X <= m_RightDown.x(); X++)
		{
			std::map<float, int> DepthAndIndices;

			Eigen::Vector4d PixelPosition = { X / m_WindowSize.x() * 2 - 1, Y / m_WindowSize.y() * 2 - 1, 0.0f, 1.0f };

			PixelPosition = PVInverse * PixelPosition;
			PixelPosition /= PixelPosition.w();

			Eigen::Vector3f RayOrigin = m_ViewPos;
			Eigen::Vector3f RayDirection = Eigen::Vector3f(PixelPosition) - RayOrigin;
			RayDirection /= RayDirection.norm();

			for (int i = 0; i < vioPointIndices.size(); i++)
			{
				Eigen::Vector3f Pos{ Scene.getPositionAt(i) };
				Eigen::Vector3f Normal{ Scene.getNormalAt(i) };

				float K = (Pos - RayOrigin).dot(Normal) / RayDirection.dot(Normal);

				Eigen::Vector3f IntersectPosition = RayOrigin + K * RayDirection;

				const float SurfelRadius = 1.0f;	//surfel world radius

				if ((IntersectPosition - Pos).norm() < SurfelRadius)
					DepthAndIndices[K] = vioPointIndices[i];
			}

			int Offset = X - m_LeftUp.x() + (Y - m_LeftUp.y()) * AreaWidth;
			_ASSERTE(Offset >= 0);

			const float WorldLengthLimit = 0.5f;	//magic
			if (Offset < PointsDepth.size() && !DepthAndIndices.empty())
			{
				auto MinDepth = DepthAndIndices.begin()->first;
				for (auto& Pair : DepthAndIndices)
				{
					if (Pair.first - MinDepth < WorldLengthLimit)
					{
						Mutex.lock();
						ResultPoints.insert(Pair.second);
						Mutex.unlock();
					}

				}
			}
		}
	}

	vioPointIndices = { ResultPoints.begin(), ResultPoints.end() };
}

//*****************************************************************
//FUNCTION: 
void CScreenSpaceOperation::cullByRadius(std::vector<pcl::index_t>& vioPointIndices, std::vector<float>& voPointDistance, float vRadius, const hiveConfig::CHiveConfig* vClusterConfig)
{
	auto& Scene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	static int AreaWidth = m_RightDown.x() - m_LeftUp.x() + 1;
	static int AreaHeight = m_RightDown.y() - m_LeftUp.y() + 1;

	std::vector<pcl::index_t> ResultPoints;
	voPointDistance.clear();

	Eigen::Vector2i CircleCenter = { m_LeftUp.x() + 0.5 * AreaWidth, m_LeftUp.y() + 0.5 * AreaHeight };

	for (auto Index : vioPointIndices)
	{
		auto Position = Scene.getPositionAt(Index);
		Position = m_PvMatrix.cast<float>() * Position;
		Position /= Position.eval().w();
		Position += Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
		Position /= 2.0f;
		Position.x() *= m_WindowSize.x();
		Position.y() *= m_WindowSize.y();

		Eigen::Vector2i ScreenPos{ Position.x(), Position.y() };
		float Distance = (ScreenPos - CircleCenter).norm();

		if (Distance <= vRadius)
		{
			ResultPoints.push_back(Index);
			voPointDistance.push_back(Distance);
		}

	}

	vioPointIndices = ResultPoints;
}

//*****************************************************************
//FUNCTION: 
void CScreenSpaceOperation::cull(std::vector<pcl::index_t>& vioPointIndices, std::vector<float>& voPointDistance, const hiveConfig::CHiveConfig* vClusterConfig)
{
	return;
}