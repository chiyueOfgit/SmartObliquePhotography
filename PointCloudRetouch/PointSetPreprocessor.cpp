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

	const Eigen::Vector2i Resolution = { 128, 128 };
	const Eigen::Vector2f SampleDeltaNDC = { static_cast<float>(MaxPos.x() - MinPos.x()) / Resolution.x(), static_cast<float>(MaxPos.y() - MinPos.y()) / Resolution.y() };

	std::vector<float> PointsDepth(Resolution.x() * Resolution.y(), FLT_MAX);
	std::set<pcl::index_t> ResultPoints;

	Eigen::Matrix4d PVInverse = vPvMatrix.inverse();

	std::mutex Mutex;

#pragma omp parallel for
	for (int i = 0; i < Resolution.y(); i++)
	{
		float Y = MinPos.y() + (i + 0.5f) * SampleDeltaNDC.y();

		for (int k = 0; k < Resolution.x(); k++)
		{
			float X = MinPos.x() + (k + 0.5f) * SampleDeltaNDC.x();

			std::map<float, int> DepthAndIndices;

			Eigen::Vector4d PixelPosition = { X, Y, 0.0f, 1.0f };

			PixelPosition = PVInverse * PixelPosition;
			PixelPosition /= PixelPosition.w();

			Eigen::Vector3f RayOrigin = vViewPos;
			Eigen::Vector3f RayDirection = Eigen::Vector3f(PixelPosition) - RayOrigin;
			RayDirection /= RayDirection.norm();

			for (int i = 0; i < vioPointSet.size(); i++)
			{
				Eigen::Vector3f Pos{ CloudScene.getPositionAt(i) };
				Eigen::Vector3f Normal{ CloudScene.getNormalAt(i) };

				float K = (Pos - RayOrigin).dot(Normal) / RayDirection.dot(Normal);

				Eigen::Vector3f IntersectPosition = RayOrigin + K * RayDirection;

				const float SurfelRadius = 1.0f;	//surfel world radius

				if ((IntersectPosition - Pos).norm() < SurfelRadius)
					DepthAndIndices[K] = vioPointSet[i];
			}

			int Offset = k + i * Resolution.x();
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

	vioPointSet = { ResultPoints.begin(), ResultPoints.end() };
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