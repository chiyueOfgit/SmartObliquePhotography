#include "pch.h"
#include "PointSetPreprocessor.h"
#include "PointCloudRetouchManager.h"
#include <omp.h>

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION:
void CPointSetPreprocessor::cullByDepth(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const Eigen::Vector3d& vViewPos)
{
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	auto [MinPos, MaxPos] = __computeBoundingBoxOnNdc(vioPointSet, vPvMatrix);

	const Eigen::Vector2i Resolution = { 100, 100 };
	const Eigen::Vector2d SampleDeltaNDC = { (MaxPos.x() - MinPos.x()) / Resolution.x(), (MaxPos.y() - MinPos.y()) / Resolution.y() };

	std::vector<float> PointsDepth(Resolution.x() * Resolution.y(), FLT_MAX);
	std::set<pcl::index_t> ResultPoints;

	Eigen::Matrix4d PVInverse = vPvMatrix.inverse();

	std::mutex Mutex;

#pragma omp parallel for
	for (int i = 0; i < Resolution.y(); i++)
	{
		double Y = MinPos.y() + (i + 0.5) * SampleDeltaNDC.y();

		for (int k = 0; k < Resolution.x(); k++)
		{
			double X = MinPos.x() + (k + 0.5) * SampleDeltaNDC.x();

			std::map<double, int> DepthAndIndices;

			Eigen::Vector4d PixelPosition = { X, Y, 0.0, 1.0 };

			PixelPosition = PVInverse * PixelPosition;
			PixelPosition /= PixelPosition.w();

			Eigen::Vector3d RayOrigin = vViewPos;
			Eigen::Vector3d RayDirection = { PixelPosition.x() - RayOrigin.x(), PixelPosition.y() - RayOrigin.y(), PixelPosition.z() - RayOrigin.z() };
			RayDirection /= RayDirection.norm();

			for (int i = 0; i < vioPointSet.size(); i++)
			{
				Eigen::Vector4f Pos4f = CloudScene.getPositionAt(i);
				Eigen::Vector3d Pos = { (double)Pos4f.x(), (double)Pos4f.y() ,(double)Pos4f.z() };
				//Eigen::Vector4f Normal4f = CloudScene.getNormalAt(i);
				//Eigen::Vector3f Normal = { Normal4f.x(), Normal4f.y(), Normal4f.z() };
				Eigen::Vector3d Normal = -RayDirection;

				double K = (Pos - RayOrigin).dot(Normal) / RayDirection.dot(Normal);

				Eigen::Vector3d IntersectPosition = RayOrigin + K * RayDirection;

				const double SurfelRadius = 100.0;	//surfel world radius

				if ((IntersectPosition - Pos).norm() < SurfelRadius)
					DepthAndIndices[K] = vioPointSet[i];
			}

			int Offset = k + i * Resolution.x();
			_ASSERTE(Offset >= 0);

			const double WorldLengthLimit = 10.0;	//magic
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
			
			return vSignedDistanceFunc({ Position.x(), Position.y() }) > 0.0f;
		}), vioPointSet.end());
}

//*****************************************************************
//FUNCTION:
std::pair<Eigen::Vector2d, Eigen::Vector2d> CPointSetPreprocessor::__computeBoundingBoxOnNdc(const std::vector<pcl::index_t>& vPointSet, const Eigen::Matrix4d& vPvMatrix)
{
	//TODO: 用PvMatrix从物体空间转Ndc这个过程可以提出来
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	Eigen::Vector2d MinPos(DBL_MAX, DBL_MAX);
	Eigen::Vector2d MaxPos(-DBL_MAX, -DBL_MAX);
	for (auto& i : vPointSet)
	{
		Eigen::Vector4d Position;
		Position = vPvMatrix * CloudScene.getPositionAt(i).cast<double>();
		Position /= Position.eval().w();

		Eigen::Vector2d NdCoord(Position.x(), Position.y());

		if (MinPos.x() > NdCoord.x())
			MinPos.x() = NdCoord.x();
		if (MinPos.y() > NdCoord.y())
			MinPos.y() = NdCoord.y();
		if (MaxPos.x() < NdCoord.x())
			MaxPos.x() = NdCoord.x();
		if (MaxPos.y() < NdCoord.y())
			MaxPos.y() = NdCoord.y();
	}

	return { Eigen::Vector2d{}, Eigen::Vector2d{} };
}