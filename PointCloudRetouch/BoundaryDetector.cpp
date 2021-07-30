#include "pch.h"
#include "BoundaryDetector.h"
#include "map"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "PointCloudRetouchManager.h"

#define PI 3.1415926
using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CBoundaryDetector, KEYWORD::BOUNDARY_DETECTOR)

//*****************************************************************
//FUNCTION: 
void CBoundaryDetector::runV(std::vector<pcl::index_t>& vBoundarySet, std::vector<std::vector<pcl::index_t>>& voHoleSet, const hiveConfig::CHiveConfig* vConfig)
{
	if (vBoundarySet.empty())
		return;
	auto pManager = CPointCloudRetouchManager::getInstance();
	for (auto CurrentIndex : vBoundarySet)
		if (CurrentIndex < 0 || CurrentIndex >= pManager->getRetouchScene().getNumPoint())
			_THROW_RUNTIME_ERROR("Index is out of range");

	std::vector<pcl::index_t> BoundarySet;
	
	for(auto Index: vBoundarySet)
	{
		auto HomoCenterPosition = pManager->getRetouchScene().getPositionAt(Index);
		auto HomoCenterNormal = pManager->getRetouchScene().getNormalAt(Index);
		Eigen::Vector3f CenterPosition{ HomoCenterPosition.x(), HomoCenterPosition.y(), HomoCenterPosition.z() };
		Eigen::Vector3f CenterNormal{ HomoCenterNormal.x(), HomoCenterNormal.y(), HomoCenterNormal.z() };
		CenterNormal.normalize();

		auto NeighborSet = pManager->buildNeighborhood(Index);
		//FitNormalÔÝÊ±ÓÃCenterNormal´úÌæ
		Eigen::Vector3f FitNormal = CenterNormal;
		
		auto HomoStandardPos = pManager->getRetouchScene().getPositionAt(NeighborSet[1]);
		Eigen::Vector3f StandardPos{ HomoStandardPos.x(), HomoStandardPos.y(), HomoStandardPos.z() };
		auto StandardProjectivePos = __calcProjectivePoint(CenterPosition, FitNormal, StandardPos);
		Eigen::Vector3f StandardVector = StandardProjectivePos - CenterPosition;
		StandardVector.normalize();
		
		std::vector<float> AngleSet;
		int Sum = 0;
		for(int i = 1;i < NeighborSet.size();i++)
		{
			auto HomoNeighborPos = pManager->getRetouchScene().getPositionAt(NeighborSet[i]);
			Eigen::Vector3f NeighborPos{ HomoNeighborPos.x(), HomoNeighborPos.y(), HomoNeighborPos.z() };
			auto ProjectivePos = __calcProjectivePoint(CenterPosition, FitNormal, NeighborPos);
			Eigen::Vector3f TempVector = ProjectivePos - CenterPosition;
			TempVector.normalize();
			auto Angle = __calcAngle(StandardVector, TempVector, FitNormal);
			AngleSet.push_back(Angle);
		}
		
		sort(AngleSet.begin(), AngleSet.end());
		int Size = AngleSet.size();
		if (2 * PI - AngleSet[Size - 1] > PI / 2 && 2 * PI - AngleSet[Size - 1] < PI)
			Sum++;
		else if(2 * PI - AngleSet[Size - 1] > PI)
			continue;
		for (int k = 1; k < Size; k++)
		{
			if (AngleSet[k] - AngleSet[k - 1] > PI / 2)
				Sum++;
			else if(AngleSet[k] - AngleSet[k - 1] > PI )
			{
				Sum = 0;
				break;
			}
		}
		if (Sum == 1)
			BoundarySet.push_back(Index);
	}
	
	__divideBoundary(BoundarySet, voHoleSet);
	for(auto& Hole: voHoleSet)
		for(auto Index: Hole)
			pManager->tagPointLabel(Index, EPointLabel::KEPT, 0, 0);
}

Eigen::Vector3f CBoundaryDetector::__calcProjectivePoint(Eigen::Vector3f& vCenterPosition, Eigen::Vector3f& vCenterNormal, Eigen::Vector3f& vProjectPosition)
{
	Eigen::Vector3f DiffVector = vProjectPosition - vCenterPosition;
	float Distance = DiffVector.dot(vCenterNormal);
	
	return vProjectPosition -  vCenterNormal * Distance;
}

float CBoundaryDetector::__calcAngle(Eigen::Vector3f& vStandardVector, Eigen::Vector3f& vOtherVector, Eigen::Vector3f& vCenterNormal)
{
	auto Dot = vStandardVector.dot(vOtherVector);
	if (Dot > 1.0f)
		Dot = 1.0f;
	else if (Dot < -1.0f)
		Dot = -1.0f;
	float Angle = std::acos(Dot);
	auto VectorCross = vStandardVector.cross(vOtherVector);
	VectorCross.normalize();
	if (VectorCross.dot(vCenterNormal) >= 0)
		return Angle;
	else
		return 2 * PI - Angle;
}

void CBoundaryDetector::__calcFitPlaneLeastSquares(Eigen::Vector3f& voPlaneCoeff, const std::vector<Eigen::Vector3f>& vData)
{
	int Size = voPlaneCoeff.size();
	if (Size < 3)
		return;

	float MeanX = 0, MeanY = 0, MeanZ = 0;
	float MeanXX = 0, MeanYY = 0, MeanZZ = 0;
	float MeanXY = 0, MeanXZ = 0, MeanYZ = 0;
	for (int i = 0; i < Size; i++)
	{
		MeanX += vData[i].x();
		MeanY += vData[i].y();
		MeanZ += vData[i].z();
		
		MeanXX += vData[i].x() * vData[i].x();
		MeanYY += vData[i].y() * vData[i].y();
		MeanZZ += vData[i].z() * vData[i].z();
										   
		MeanXY += vData[i].x() * vData[i].y();
		MeanXZ += vData[i].x() * vData[i].z();
		MeanYZ += vData[i].y() * vData[i].z();
	}
	MeanX /= Size;
	MeanY /= Size;
	MeanZ /= Size;
	MeanXX /= Size;
	MeanYY /= Size;
	MeanZZ /= Size;
	MeanXY /= Size;
	MeanXZ /= Size;
	MeanYZ /= Size;

	/* eigenvector */
	Eigen::Matrix3f ParaMat;
	ParaMat(0, 0) = MeanXX - MeanX * MeanX; ParaMat(0, 1) = MeanXY - MeanX * MeanY; ParaMat(0, 2) = MeanXZ - MeanX * MeanZ;
	ParaMat(1, 0) = MeanXY - MeanX * MeanY; ParaMat(1, 1) = MeanYY - MeanY * MeanY; ParaMat(1, 2) = MeanYZ - MeanY * MeanZ;
	ParaMat(2, 0) = MeanXZ - MeanX * MeanZ; ParaMat(2, 1) = MeanYZ - MeanY * MeanZ; ParaMat(2, 2) = MeanZZ - MeanZ * MeanZ;
	Eigen::EigenSolver<Eigen::Matrix3f> EigenMat(ParaMat);
	Eigen::Matrix3f EigenValue = EigenMat.pseudoEigenvalueMatrix();
	Eigen::Matrix3f EigenVector = EigenMat.pseudoEigenvectors();

	/* the eigenvector corresponding to the minimum eigenvalue */
	float Value1 = EigenValue(0, 0);
	float Value2 = EigenValue(1, 1);
	float Value3 = EigenValue(2, 2);
	int MinNumber = 0;
	if ((abs(Value2) <= abs(Value1)) && (abs(Value2) <= abs(Value3)))
	{
		MinNumber = 1;
	}
	if ((abs(Value3) <= abs(Value1)) && (abs(Value3) <= abs(Value2)))
	{
		MinNumber = 2;
	}
	float A = EigenVector(0, MinNumber);
	float B = EigenVector(1, MinNumber);
	float C = EigenVector(2, MinNumber);
	float D = -(A * MeanX + B * MeanY + C * MeanZ);

	if (C < 0)
	{
		A *= -1.0f;
		B *= -1.0f;
		C *= -1.0f;
		D *= -1.0f;
	}
	
	voPlaneCoeff.x() = A;
	voPlaneCoeff.y() = B;
	voPlaneCoeff.z() = C;
}

Eigen::Vector3f CBoundaryDetector::__calcFitPlaneRANSAC(const std::vector<Eigen::Vector3f>& vData, double vDistanceThreshold, const Eigen::Vector3f& vUp)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(Eigen::Vector3f Normal: vData)
	{
		pcl::PointXYZ Point;
		Point.x = Normal.x();
		Point.y = Normal.y();
		Point.z = Normal.z();
		vCloud->push_back(Point);
	}

	Eigen::VectorXf Coeff;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr ModelPlane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(vCloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> Ransac(ModelPlane);
	Ransac.setDistanceThreshold(vDistanceThreshold);
	Ransac.computeModel();
	Ransac.getModelCoefficients(Coeff);
	if (!Coeff.size())
		return { 0, 0, 0};
	const Eigen::Vector3f Normal(Coeff.x(), Coeff.y(), Coeff.z());
	if (Normal.dot(vUp) < 0.0f)
		Coeff *= -1.0f;
	
	Eigen::Vector3f TempCoeff{ Coeff.x(), Coeff.y(), Coeff.z()};
	return TempCoeff;
}

void CBoundaryDetector::__divideBoundary(std::vector<pcl::index_t>& vBoundaryPointSet, std::vector<std::vector<pcl::index_t>>& voHoleSet)
{
    std::map<int, bool> TraversedFlag;
	for (auto Index : vBoundaryPointSet)
	{
		TraversedFlag.insert(std::make_pair(Index, false));
	}
	std::queue<pcl::index_t> ExpandingCandidateQueue;
	
	while (!vBoundaryPointSet.empty())
	{
		ExpandingCandidateQueue.push(vBoundaryPointSet[0]);

		std::vector<pcl::index_t> TempBoundary;
		while (!ExpandingCandidateQueue.empty())
		{
			
			pcl::index_t Candidate = ExpandingCandidateQueue.front();
			ExpandingCandidateQueue.pop();
			
			TempBoundary.push_back(Candidate);
			
			std::vector<pcl::index_t> NeighborSet;
			__findNearestBoundaryPoint(Candidate, vBoundaryPointSet, NeighborSet);
			if(NeighborSet.empty())
				continue;
			
			for(auto Index:NeighborSet)
			{
				if (TraversedFlag[Index] == true)
				    continue;
			    else
			    {
				    TraversedFlag[Index] = true;
				    ExpandingCandidateQueue.push(Index);
			    }
			}
		}
		if (TempBoundary.size() > 20)
			voHoleSet.push_back(TempBoundary);
		
		for (auto Iter = vBoundaryPointSet.begin(); Iter != vBoundaryPointSet.end(); )
		{
			if (TraversedFlag[*Iter] == true)
				Iter = vBoundaryPointSet.erase(Iter);
			else
				++Iter;
		}
		
	}
}

void CBoundaryDetector::__findNearestBoundaryPoint(pcl::index_t vSeed, std::vector<pcl::index_t>& vTotalSet, std::vector<pcl::index_t>& voNeighborSet)
{
	auto pManager = CPointCloudRetouchManager::getInstance();
	auto SeedPos = pManager->getRetouchScene().getPositionAt(vSeed);
	for(auto Index: vTotalSet)
	{
		auto TempPos = pManager->getRetouchScene().getPositionAt(Index);
		if ((SeedPos - TempPos).norm() < 0.5)
			voNeighborSet.push_back(Index);
	}
}