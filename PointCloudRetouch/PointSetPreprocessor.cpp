#include "pch.h"
#include "PointSetPreprocessor.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchManager.h"
#include <omp.h>

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION:
void CPointSetPreprocessor::cullByDepth(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const Eigen::Vector3d& vViewPos)
{
	if (vioPointSet.empty())
		return;

	auto *pManager = CPointCloudRetouchManager::getInstance();
	for(auto Iter = vioPointSet.begin(); Iter != vioPointSet.end(); )
	{
		std::size_t TempLabel;
		pManager->dumpPointLabelAt(TempLabel, *Iter);
		if (static_cast<EPointLabel>(TempLabel) == EPointLabel::DISCARDED)
			Iter = vioPointSet.erase(Iter);
		else
			++Iter;
	}
	
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	auto [MinPos, MaxPos] = __computeBoundingBoxOnNdc(vioPointSet, vPvMatrix);

	int NumSqrtPoints = static_cast<int>(sqrt(vioPointSet.size()));

	const Eigen::Vector2i TileResolution = { int(0.1 * NumSqrtPoints) + 1, int(0.1 * NumSqrtPoints) + 1 };
	const Eigen::Vector2d TileDeltaNDC = { (MaxPos.x() - MinPos.x()) / TileResolution.x(), (MaxPos.y() - MinPos.y()) / TileResolution.y() };
	std::vector<std::vector<pcl::index_t>> TileData(TileResolution.x() * TileResolution.y());

	std::mutex Mutex;

#pragma omp parallel for
	for (int i = 0; i < vioPointSet.size(); i++)
	{
		auto Position = CloudScene.getPositionAt(vioPointSet[i]);
		Position = vPvMatrix.cast<float>() * Position;
		Position /= Position.w();

		Eigen::Vector2i TileCoord;
		TileCoord.x() = static_cast<int>((Position.x() - MinPos.x()) / TileDeltaNDC.x());
		TileCoord.y() = static_cast<int>((Position.y() - MinPos.y()) / TileDeltaNDC.y());

		if (TileCoord.x() >= 0 && TileCoord.x() < TileResolution.x() && TileCoord.y() >= 0 && TileCoord.y() < TileResolution.y())
		{
			Mutex.lock();
			TileData[TileCoord.x() + TileCoord.y() * TileResolution.x()].push_back(vioPointSet[i]);
			Mutex.unlock();
		}
	}

	const Eigen::Vector2i Resolution = { NumSqrtPoints, NumSqrtPoints };
	const Eigen::Vector2d SampleDeltaNDC = { (MaxPos.x() - MinPos.x()) / Resolution.x(), (MaxPos.y() - MinPos.y()) / Resolution.y() };

	std::map<pcl::index_t, double> ResultRecords;
	std::set<pcl::index_t> ResultPoints;

	Eigen::Matrix4d PVInverse = vPvMatrix.inverse();

#pragma omp parallel for
	for (int i = 0; i < Resolution.y(); i++)
	{
		double Y = MinPos.y() + (i + 0.5) * SampleDeltaNDC.y();

		for (int k = 0; k < Resolution.x(); k++)
		{
			double X = MinPos.x() + (k + 0.5) * SampleDeltaNDC.x();

			std::map<double, pcl::index_t> DepthAndIndices;

			Eigen::Vector4d PixelPosition = { X, Y, 0.0, 1.0 };

			PixelPosition = PVInverse * PixelPosition;
			PixelPosition /= PixelPosition.w();

			Eigen::Vector3d RayOrigin = vViewPos;
			Eigen::Vector3d RayDirection = { PixelPosition.x() - RayOrigin.x(), PixelPosition.y() - RayOrigin.y(), PixelPosition.z() - RayOrigin.z() };
			RayDirection /= RayDirection.norm();

			Eigen::Vector2i TileCoord;
			TileCoord.x() = static_cast<int>((X - MinPos.x()) / TileDeltaNDC.x());
			TileCoord.y() = static_cast<int>((Y - MinPos.y()) / TileDeltaNDC.y());
			auto& Tile = TileData[TileCoord.x() + TileCoord.y() * TileResolution.x()];

			for (auto Index : Tile)
			{
				Eigen::Vector4f Pos4f = CloudScene.getPositionAt(Index);
				Eigen::Vector3d Pos = { (double)Pos4f.x(), (double)Pos4f.y() ,(double)Pos4f.z() };
				Eigen::Vector4f Normal4f = CloudScene.getNormalAt(i);
				Eigen::Vector3d Normal = { Normal4f.x(), Normal4f.y(), Normal4f.z() };

				double Depth = (Pos - RayOrigin).dot(Normal) / RayDirection.dot(Normal);

				Eigen::Vector3d IntersectPosition = RayOrigin + Depth * RayDirection;
				
				const double SurfelRadius = 3.0;	//surfel world radius

				if ((IntersectPosition - Pos).norm() < SurfelRadius)
				{
					DepthAndIndices[Depth] = Index;
				}
			}

			const double WorldLengthLimit = 1.0;	//magic
			if (!DepthAndIndices.empty())
			{
				auto MinDepth = DepthAndIndices.begin()->first;
				for (auto& Pair : DepthAndIndices)
				{
					if (Pair.first - MinDepth < WorldLengthLimit)
					{
						Mutex.lock();
						if (ResultRecords.find(Pair.second) == ResultRecords.end())
						{
							ResultRecords.insert({ Pair.second, Pair.first });
						}
						ResultPoints.insert(Pair.second);
						Mutex.unlock();
					}
					else
						break;

				}
			}
		}
	}

	//global culling
	double AverageK = 0.0;
	double MinK = DBL_MAX;
	double MaxK = -DBL_MAX;
	for (auto& Pair : ResultRecords)
	{
		AverageK += Pair.second;
		if (Pair.second < MinK)
			MinK = Pair.second;
		else if (Pair.second > MaxK)
			MaxK = Pair.second;
	}
	AverageK /= ResultRecords.size();
	_ASSERTE(MinK <= MaxK);
	double MidK = (MinK + MaxK) * 0.5;

	double MinRate = 0.3;
	double MaxRate = 0.3;
	if (MaxK - MinK <= 10.0)	//magic depth
		MinRate = MaxRate = 1.0;
	double ThresholdMinK = MinK * MinRate + AverageK * (1 - MinRate);
	double ThresholdMaxK = MaxK * MaxRate + AverageK * (1 - MaxRate);

	std::vector<pcl::index_t> Result;
	std::vector<pcl::index_t> OutMin;
	std::vector<pcl::index_t> OutMax;

	for (auto Index : ResultPoints)
	{
		double Depth = ResultRecords.find(Index)->second;
		if (Depth < ThresholdMinK)
			OutMin.push_back(Index);
		else if (Depth > ThresholdMaxK)
			OutMax.push_back(Index);
		else
			Result.push_back(Index);
	}
	std::vector<pcl::index_t>* pOut2Reserve = nullptr;

	pOut2Reserve = MidK > AverageK ? &OutMin : &OutMax;
	
	for (auto Index : *pOut2Reserve)
		Result.push_back(Index);

	vioPointSet = Result;
}

//*****************************************************************
//FUNCTION:
void CPointSetPreprocessor::cullBySdf(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vSignedDistanceFunc)
{	
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	vioPointSet.erase(std::remove_if(vioPointSet.begin(), vioPointSet.end(), 
		[&](auto vIndex)
		{
			Eigen::Vector4f Position = CloudScene.getPositionAt(vIndex);
			Position = vPvMatrix.cast<float>() * Position;
			Position /= Position.eval().w();
			
			return vSignedDistanceFunc({ Position.x(), Position.y() }) > 0.0;
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

	return { MinPos, MaxPos };
}