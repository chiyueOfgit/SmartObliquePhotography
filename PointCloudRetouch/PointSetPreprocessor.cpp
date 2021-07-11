#include "pch.h"
#include "PointSetPreprocessor.h"
#include "PointCloudRetouchManager.h"
#include <omp.h>

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION:
void CPointSetPreprocessor::cullByDepth(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const Eigen::Vector3f& vViewPos)
{
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	auto [MinPos, MaxPos] = __computeBoundingBoxOnNdf(vioPointSet, vPvMatrix);

//	static int AreaWidth = m_RightDown.x() - m_LeftUp.x() + 1;
//	static int AreaHeight = m_RightDown.y() - m_LeftUp.y() + 1;
//
//	std::vector<float> PointsDepth(AreaWidth * AreaHeight, FLT_MAX);
//	std::set<pcl::index_t> ResultPoints;
//
//	Eigen::Matrix4d PVInverse = m_PvMatrix.inverse();
//
//	//tile projection
//	const std::size_t NumPartitionX = 0.5 * AreaWidth, NumPartitionY = 0.5 * AreaHeight;
//	float TileDeltaX = (float)AreaWidth / NumPartitionX;
//	float TileDeltaY = (float)AreaHeight / NumPartitionY;
//
//	std::vector<std::vector<pcl::index_t>> PointTile(NumPartitionX * NumPartitionY);
//
//	std::mutex Mutex;
//
//#pragma omp parallel for
//	for (int i = 0; i < vioPointIndices.size(); i++)
//	{
//		auto Index = vioPointIndices[i];
//		Eigen::Vector4f Position = Scene.getPositionAt(Index);
//
//		Position = m_PvMatrix * Position;
//		Position /= Position.eval().w();
//		Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
//		Position /= 2.0;
//		Position.x() *= m_WindowSize.x();
//		Position.y() *= m_WindowSize.y();
//
//		int TileIndexX = (Position.x() - m_LeftUp.x()) / TileDeltaX;
//		int TileIndexY = (Position.y() - m_LeftUp.y()) / TileDeltaY;
//
//		_ASSERTE(TileIndexX >= 0 && TileIndexY >= 0);
//
//		Mutex.lock();
//		PointTile[TileIndexX + TileIndexY * NumPartitionX].push_back(Index);
//		Mutex.unlock();
//	}
//	std::mutex Mutex;
//
//#pragma omp parallel for
//	for (int Y = m_LeftUp.y(); Y <= m_RightDown.y(); Y++)
//	{
//		for (int X = m_LeftUp.x(); X <= m_RightDown.x(); X++)
//		{
//			std::map<float, int> DepthAndIndices;
//
//			Eigen::Vector4d PixelPosition = { X / m_WindowSize.x() * 2 - 1, Y / m_WindowSize.y() * 2 - 1, 0.0f, 1.0f };
//
//			PixelPosition = PVInverse * PixelPosition;
//			PixelPosition /= PixelPosition.w();
//
//			Eigen::Vector3f RayOrigin = m_ViewPos;
//			Eigen::Vector3f RayDirection = Eigen::Vector3f(PixelPosition) - RayOrigin;
//			RayDirection /= RayDirection.norm();
//
//			for (int i = 0; i < vioPointIndices.size(); i++)
//			{
//				Eigen::Vector3f Pos{ Scene.getPositionAt(i) };
//				Eigen::Vector3f Normal{ Scene.getNormalAt(i) };
//
//				float K = (Pos - RayOrigin).dot(Normal) / RayDirection.dot(Normal);
//
//				Eigen::Vector3f IntersectPosition = RayOrigin + K * RayDirection;
//
//				const float SurfelRadius = 1.0f;	//surfel world radius
//
//				if ((IntersectPosition - Pos).norm() < SurfelRadius)
//					DepthAndIndices[K] = vioPointIndices[i];
//			}
//
//			int Offset = X - m_LeftUp.x() + (Y - m_LeftUp.y()) * AreaWidth;
//			_ASSERTE(Offset >= 0);
//
//			const float WorldLengthLimit = 0.5f;	//magic
//			if (Offset < PointsDepth.size() && !DepthAndIndices.empty())
//			{
//				auto MinDepth = DepthAndIndices.begin()->first;
//				for (auto& Pair : DepthAndIndices)
//				{
//					if (Pair.first - MinDepth < WorldLengthLimit)
//					{
//						Mutex.lock();
//						ResultPoints.insert(Pair.second);
//						Mutex.unlock();
//					}
//
//				}
//			}
//		}
//	}
//
//	vioPointIndices = { ResultPoints.begin(), ResultPoints.end() };
}

//*****************************************************************
//FUNCTION:
void CPointSetPreprocessor::cullBySdf(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const std::function<float(Eigen::Vector2f)>& vSignedDistanceFunc)
{	
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	vioPointSet.erase(std::remove_if(vioPointSet.begin(), vioPointSet.end(), 
		[&](auto vIndex)
		{
			Eigen::Vector4f Position = CloudScene.getPositionAt(vIndex);
			Position = vPvMatrix.cast<float>() * Position;
			Position /= Position.eval().w();
			Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
			Position /= 2.0;
			
			return vSignedDistanceFunc({ Position.x(), Position.y() }) > 0.0f;
		}), vioPointSet.end());
}

//*****************************************************************
//FUNCTION:
std::pair<Eigen::Vector2f, Eigen::Vector2f> CPointSetPreprocessor::__computeBoundingBoxOnNdf(const std::vector<pcl::index_t>& vPointSet, const Eigen::Matrix4d& vPvMatrix)
{
	//TODO: 用PvMatrix从物体空间转Ndf这个过程可以提出来
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	Eigen::Vector2f MinPos(FLT_MAX, FLT_MAX);
	Eigen::Vector2f MaxPos(-FLT_MAX, -FLT_MAX);
	for (auto& i : vPointSet)
	{
		Eigen::Vector4f Position = CloudScene.getPositionAt(i);
		Position = vPvMatrix.cast<float>() * Position;
		Position /= Position.eval().w();
		Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
		Position /= 2.0;

		Eigen::Vector2f NdfCoord(Position.x(), Position.y());

		if (MinPos.x() > NdfCoord.x())
			MinPos.x() = NdfCoord.x();
		if (MinPos.y() > NdfCoord.y())
			MinPos.y() = NdfCoord.y();
		if (MaxPos.x() > NdfCoord.x())
			MaxPos.x() = NdfCoord.x();
		if (MaxPos.y() > NdfCoord.y())
			MaxPos.y() = NdfCoord.y();
	}

	return { MinPos, MaxPos };
}