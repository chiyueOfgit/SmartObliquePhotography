#include "pch.h"
#include "HoleRepairer.h"
#include "BoundaryDetector.h"
#include "TextureSynthesizer.h"
#include "OrderIndependentTextureSynthesizer.h"
#include "MultithreadTextureSynthesizer.h"
#include "TreeBasedTextureSynthesizer.h"
#include "PlanarityFeature.h"
#include "PointCloudRetouchManager.h"
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <pcl/common/centroid.h>

#define PI 3.1415927

using namespace hiveObliquePhotography::PointCloudRetouch;

bool CHoleRepairer::init(const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(vConfig);
	m_pConfig = vConfig;

	for (auto i = 0; i < vConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pConfig = vConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("BOUNDARY_DETECTOR")))
		{
			m_pBoundaryDetector = hiveDesignPattern::hiveGetOrCreateProduct<CBoundaryDetector>("BOUNDARY_DETECTOR");
			_ASSERTE(m_pBoundaryDetector);
			m_pBoundaryDetector->init(pConfig);
			continue;
		}
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("TEXTURE_SYNTHESIZER")))
		{
			m_pTextureConfig = pConfig;
			continue;
		}
	}
	return true;
}

void CHoleRepairer::setHoleRegion(const std::vector<pcl::index_t>& vHoleRegion)
{
    auto TempSet = vHoleRegion;
	std::vector<std::vector<pcl::index_t>> TempBoundarySet;
	m_pBoundaryDetector->execute<CBoundaryDetector>(TempSet, TempBoundarySet);
	m_BoundarySet = TempBoundarySet;
}

void CHoleRepairer::repairHole(std::vector<pcl::PointXYZRGBNormal>& voNewPoints)
{
	if (!m_BoundarySet.empty() && !m_Input.empty())
	{
		std::vector<pcl::PointXYZRGBNormal> NewPoints;
		for (auto& Boundary : m_BoundarySet)
		{
			std::vector<pcl::PointXYZRGBNormal> TempPoints;
			repairHoleByBoundaryAndInput(Boundary, m_Input, TempPoints);
			NewPoints.insert(NewPoints.end(), TempPoints.begin(), TempPoints.end());
		}
		std::swap(voNewPoints, NewPoints);

		__reset();
	}
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::repairHoleByBoundaryAndInput(const std::vector<pcl::index_t>& vBoundaryIndices, const std::vector<pcl::index_t>& vInputIndices, std::vector<pcl::PointXYZRGBNormal>& voNewPoints)
{
	auto pManager = CPointCloudRetouchManager::getInstance();
	//Input
	auto InputBox = pManager->calcOBBByIndices(vInputIndices);
	auto InputPlane = __calculatePlaneByIndices(vInputIndices, std::get<0>(InputBox));
	auto InputBoxMax = std::get<2>(InputBox);
	auto InputBoxMin = std::get<1>(InputBox);
	
	auto InputBoxLength = InputBoxMax - InputBoxMin;
	auto InputAxisOrder = __calcAxisOrder(InputPlane);
	Eigen::Vector2f InputPiece{ InputBoxMax.data()[InputAxisOrder[0]] - InputBoxMin.data()[InputAxisOrder[0]], InputBoxMax.data()[InputAxisOrder[1]] - InputBoxMin.data()[InputAxisOrder[1]] };
	float InputPieceArea = InputPiece.x() * InputPiece.y();
	float PointsPerArea = vInputIndices.size() / InputPieceArea;
	const Eigen::Vector2i InputResolution{ (int)(sqrtf(vInputIndices.size()) / 2), (int)(sqrtf(vInputIndices.size()) / 2) };

	SPlaneInfos InputPlaneInfos;
	std::vector<std::vector<SLattice>> InputPlaneLattices;
	__generatePlaneLattices(InputPlane, InputBox, InputResolution, InputPlaneInfos, InputPlaneLattices);
	__projectPoints2PlaneLattices({}, InputPlaneInfos, InputPlaneLattices);

	//Boundary
	auto BoundaryBox = pManager->calcOBBByIndices(vBoundaryIndices);
	auto BoundaryPlane = __calculatePlaneByIndices(vBoundaryIndices, std::get<0>(BoundaryBox));	
	auto BoundaryBoxMax = std::get<2>(BoundaryBox);
	auto BoundaryBoxMin = std::get<1>(BoundaryBox);
	
	auto BoundaryBoxLength = BoundaryBoxMax - BoundaryBoxMin;
	const float Delta = 0.0f;
	BoundaryBoxMin = BoundaryBoxMin - Delta * BoundaryBoxLength;
	BoundaryBoxMax = BoundaryBoxMax + Delta * BoundaryBoxLength;
	Eigen::Vector2f BoundaryPiece{ BoundaryBoxMax.data()[InputAxisOrder[0]] - BoundaryBoxMin.data()[InputAxisOrder[0]], BoundaryBoxMax.data()[InputAxisOrder[1]] - BoundaryBoxMin.data()[InputAxisOrder[1]] };
	float BoundaryPieceArea = BoundaryPiece.x() * BoundaryPiece.y();

	float HolePlanePoints = PointsPerArea * BoundaryPieceArea;
	const Eigen::Vector2i HoleResolution{ (int)sqrtf(HolePlanePoints), (int)sqrtf(HolePlanePoints) };
	SPlaneInfos BoundaryPlaneInfos;
	std::vector<std::vector<SLattice>> BoundaryPlaneLattices;
	__generatePlaneLattices(BoundaryPlane, BoundaryBox, HoleResolution, BoundaryPlaneInfos, BoundaryPlaneLattices);	//Generate plane lattice
	__projectPoints2PlaneLattices({}, BoundaryPlaneInfos, BoundaryPlaneLattices);	//Throw points into the grid with the points in the Bounding Box
	Eigen::MatrixXi Mask = __genMask((1.0f * HoleResolution.cast<float>()).cast<int>(), BoundaryPlaneLattices);	

	__inputHeightCorrection(InputPlaneLattices, BoundaryPlaneLattices);

	//Generate color
	{
		auto InputColorMatrix = __extractMatrixFromLattices<Eigen::Vector3i>(InputPlaneLattices, offsetof(SLattice, Color));
		auto BoundaryColorMatrix = __extractMatrixFromLattices<Eigen::Vector3i>(BoundaryPlaneLattices, offsetof(SLattice, Color));

		__outputImage(InputColorMatrix, "Temp/input.png");
		__outputImage(BoundaryColorMatrix, "Temp/output_before.png");
		__outputImage(Mask, "Temp/mask.png");

		CTreeBasedTextureSynthesizer<int, 3> ColorSynthesizer;
		ColorSynthesizer.execute(InputColorMatrix, Mask, BoundaryColorMatrix);
		__fillLatticesByMatrix<Eigen::Vector3i>(BoundaryColorMatrix, BoundaryPlaneLattices, offsetof(SLattice, Color));

		__outputImage(BoundaryColorMatrix, "Temp/output_after.png");
	}

	//Generate height
	{
		auto InputHeightMatrix = __extractMatrixFromLattices<Eigen::Matrix<float, 1, 1>>(InputPlaneLattices, offsetof(SLattice, Height));
		auto BoundaryHeightMatrix = __extractMatrixFromLattices<Eigen::Matrix<float, 1, 1>>(BoundaryPlaneLattices, offsetof(SLattice, Height));

		__outputImage(InputHeightMatrix, "Temp/inputH.png");
		__outputImage(BoundaryHeightMatrix, "Temp/output_beforeH.png");

		CTreeBasedTextureSynthesizer<float, 1> HeightSynthesizer;
		HeightSynthesizer.execute(InputHeightMatrix, Mask, BoundaryHeightMatrix);
		__gaussBlurbyHeightMatrix(BoundaryHeightMatrix, BoundaryPlaneLattices);

		__outputImage(BoundaryHeightMatrix, "Temp/output_afterH.png");
	}

	__generateNewPointsFromLattices(BoundaryPlaneInfos.Normal, Mask, BoundaryPlaneLattices, voNewPoints);
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__generatePlaneLattices(const Eigen::Vector4f& vPlane, const std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f>& vBox, const Eigen::Vector2i& vResolution, SPlaneInfos& voPlaneInfos, std::vector<std::vector<SLattice>>& voPlaneLattices)
{
	std::vector<std::vector<SLattice>> PlaneLattices(vResolution.y(), std::vector<SLattice>(vResolution.x()));	//row-major

	Eigen::Vector3f PlaneNormal{ vPlane.x(), vPlane.y(), vPlane.z() };
	
	auto AxisOrder = __calcAxisOrder(vPlane);
	auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];

	auto BoxMax = std::get<2>(vBox);
	auto BoxMin = std::get<1>(vBox);
	
	Eigen::Vector3f BoxLength = BoxMax - BoxMin;
	float LatticeWidth = BoxLength.data()[X] / vResolution.x();
	float LatticeHeight = BoxLength.data()[Y] / vResolution.y();

	Eigen::Vector3f PlaneCenter;
	PlaneCenter.data()[X] = 0.5f * ( BoxMin.data()[X] + BoxMax.data()[X]);
	PlaneCenter.data()[Y] = 0.5f * ( BoxMin.data()[Y] + BoxMax.data()[Y]);
	PlaneCenter.data()[Z] = -(vPlane.w() + vPlane.data()[X] * PlaneCenter.data()[X] + vPlane.data()[Y] * PlaneCenter.data()[Y]) / vPlane.data()[Z];
	
	for (int Id = 0; Id < vResolution.x() * vResolution.y(); Id++)
	{
		Eigen::Vector2i LatticeCoord = { Id % vResolution.x(), int(Id / vResolution.x()) };
		Eigen::Vector3f LatticeWorldPos;
		LatticeWorldPos.data()[X] = BoxMin.data()[X] + (LatticeCoord.x() + 0.5f) * LatticeWidth;
		LatticeWorldPos.data()[Y] = BoxMin.data()[Y] + (LatticeCoord.y() + 0.5f) * LatticeHeight;
		LatticeWorldPos.data()[Z] = -(vPlane.w() + vPlane.data()[X] * LatticeWorldPos.data()[X] + vPlane.data()[Y] * LatticeWorldPos.data()[Y]) / vPlane.data()[Z];
		PlaneLattices[LatticeCoord.y()][LatticeCoord.x()].CenterPos = LatticeWorldPos;	//row-major
	}

	voPlaneInfos.AxisOrder = AxisOrder;
	voPlaneInfos.BoundingBox = vBox;
	voPlaneInfos.LatticeSize = { LatticeWidth, LatticeHeight };
	voPlaneInfos.Normal = PlaneNormal;
	voPlaneInfos.PlaneCenter = PlaneCenter;
	std::swap(voPlaneLattices, PlaneLattices);
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__projectPoints2PlaneLattices(const std::vector<pcl::index_t>& vIndices, SPlaneInfos& vioPlaneInfos, std::vector<std::vector<SLattice>>& vioPlaneLattices)
{
	_ASSERTE(!vioPlaneLattices.empty());
	auto Scene = CPointCloudRetouchManager::getInstance()->getScene();
	Eigen::Matrix3f RotationMatrix = std::get<0>(vioPlaneInfos.BoundingBox);
	Eigen::Vector2i Resolution = { vioPlaneLattices.front().size(), vioPlaneLattices.size() };
	auto X = vioPlaneInfos.AxisOrder[0], Y = vioPlaneInfos.AxisOrder[1];
	std::vector<pcl::index_t> ProjectSet;
	
	if (vIndices.empty())
	{
		//magic
		const float DeltaBoxRate = 0.1f;
		Eigen::Vector3f DeltaBox = (std::get<2>(vioPlaneInfos.BoundingBox) - std::get<1>(vioPlaneInfos.BoundingBox)) * DeltaBoxRate;
		DeltaBox.data()[vioPlaneInfos.AxisOrder[2]] += 5.0f;

		std::pair<Eigen::Vector3f, Eigen::Vector3f> ExpandBox = { std::get<1>(vioPlaneInfos.BoundingBox) - DeltaBox, std::get<2>(vioPlaneInfos.BoundingBox) + DeltaBox };
		ProjectSet = Scene.getPointsInBox(ExpandBox, RotationMatrix);
	}
	else
		ProjectSet = vIndices;
	_ASSERTE(!ProjectSet.empty());
	
	const float LatticeLengthX = (vioPlaneLattices[0][1].CenterPos - vioPlaneLattices[0][0].CenterPos).norm();
	const float LatticeLengthY = (vioPlaneLattices[1][0].CenterPos - vioPlaneLattices[0][0].CenterPos).norm();
	const float Radius = Eigen::Vector2f{ LatticeLengthX, LatticeLengthY }.norm();

	const int DeltaX2Check = (int)(Radius / LatticeLengthX) + 1;
	const int DeltaY2Check = (int)(Radius / LatticeLengthY) + 1;

	for (auto Index : ProjectSet)
	{
		auto Pos4f = Scene.getPositionAt(Index);
		Eigen::Vector3f PointPos{ Pos4f.x(), Pos4f.y(), Pos4f.z() };
		PointPos = RotationMatrix * PointPos;
		Eigen::Vector3f VecCenter2Point = PointPos - vioPlaneInfos.PlaneCenter;
		Eigen::Vector3f VecProj2Point = VecCenter2Point.dot(vioPlaneInfos.Normal) * vioPlaneInfos.Normal;
		Eigen::Vector3f ProjPoint = vioPlaneInfos.PlaneCenter + (VecCenter2Point - VecProj2Point);

		Eigen::Vector2i LatticeCoord = { (ProjPoint.data()[X] - std::get<1>(vioPlaneInfos.BoundingBox).data()[X]) / vioPlaneInfos.LatticeSize.x(), (ProjPoint.data()[Y] - std::get<1>(vioPlaneInfos.BoundingBox).data()[Y]) / vioPlaneInfos.LatticeSize.y() };
		for (int i = LatticeCoord.y() - DeltaY2Check; i <= LatticeCoord.y() + DeltaY2Check; i++)
		{
			for (int k = LatticeCoord.x() - DeltaX2Check; k <= LatticeCoord.x() + DeltaX2Check; k++)
			{
				if (i >= 0 && k >= 0 && i < Resolution.y() && k < Resolution.x() && (vioPlaneLattices[i][k].CenterPos - ProjPoint).norm() <= Radius)
					vioPlaneLattices[i][k].Indices.push_back(Index);
			}
		}
	}
	//Change back to world coordinates
	__restoreWorldCoord(vioPlaneInfos, vioPlaneLattices, RotationMatrix);
	__fillLatticesOriginInfos(vioPlaneInfos.Normal, vioPlaneLattices);
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__fillLatticesOriginInfos(const Eigen::Vector3f& vPlaneNormal, std::vector<std::vector<SLattice>>& vioPlaneLattices)
{
	_ASSERTE(!vioPlaneLattices.empty());
	auto Scene = CPointCloudRetouchManager::getInstance()->getScene();

	Eigen::Vector2i Resolution = { vioPlaneLattices.front().size(), vioPlaneLattices.size() };
	for (int Y = 0; Y < Resolution.y(); Y++)
	{
		for (int X = 0; X < Resolution.x(); X++)
		{
			auto& Lattice = vioPlaneLattices[Y][X];
			if (!Lattice.Indices.empty())
			{
				std::vector<std::pair<std::size_t, float>> ColorLengths;
				float AverageHeight = 0.0f;
				for (auto Index : Lattice.Indices)
				{
					auto TempPos = Scene.getPositionAt(Index);
					Eigen::Vector3f Pos{ TempPos.x(), TempPos.y(), TempPos.z() };
					ColorLengths.push_back({ Index, Scene.getPositionAt(Index).norm() });
					AverageHeight += (Pos - Lattice.CenterPos).dot(vPlaneNormal);
				}

				std::sort(ColorLengths.begin(), ColorLengths.end(), [](std::pair<std::size_t, float> vLeft, std::pair<std::size_t, float> vRight)
					{
						return vLeft.second < vRight.second;
					});

				Lattice.Color = Scene.getColorAt(ColorLengths[ColorLengths.size() * 0.5f].first);	//ÖÐÎ»
				Lattice.Height(0, 0) = AverageHeight / Lattice.Indices.size();
			}
		}
	}
	//post process
	__fixTextureColorAndHeight(vioPlaneLattices, Resolution.x() / 30);
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__fixTextureColorAndHeight(std::vector<std::vector<SLattice>>& vioPlaneLattices, int vKernelSize)
{
	Eigen::Vector2i Resolution = { vioPlaneLattices.front().size(), vioPlaneLattices.size() };
	auto Lattices = vioPlaneLattices;
	const int Delta = vKernelSize / 2;
	for (int Y = 0; Y < Resolution.y(); Y++)
	{
		for (int X = 0; X < Resolution.x(); X++)
		{
			auto& Lattice = Lattices[Y][X];
			if (!Lattice.Color.norm() && !Lattice.Height(0, 0))
			{
				std::vector<Eigen::Vector3i> Colors;
				float SumHeights = 0.0f;
				std::size_t NumNoValue = 0;
				for (int i = Y - Delta; i <= Y + Delta; i++)
					for (int k = X - Delta; k <= X + Delta; k++)
						if (k >= 0 && k < Resolution.x() && i >= 0 && i < Resolution.y())
						{
							auto& Neighbor = vioPlaneLattices[i][k];
							if (Neighbor.Color.norm() && Neighbor.Height(0, 0))
							{
								Colors.push_back(Neighbor.Color);
								SumHeights += Neighbor.Height(0, 0);
							}
							else
								NumNoValue++;
						}
				std::sort(Colors.begin(), Colors.end(), [](Eigen::Vector3i vLeft, Eigen::Vector3i vRight)
					{
						return vLeft.norm() < vRight.norm();
					});

				if (!Colors.empty() && NumNoValue < pow(vKernelSize, 2) / 3)
				{
					vioPlaneLattices[Y][X].Color = Colors[Colors.size() / 2];
					vioPlaneLattices[Y][X].Height(0, 0) = SumHeights / (pow(vKernelSize, 2) - NumNoValue);
				}
			}
		}
	}
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__inputHeightCorrection(std::vector<std::vector<SLattice>>& vInput, const std::vector<std::vector<SLattice>>& vBoundary)
{
	const Eigen::Vector2i InputResolution{ vInput.front().size(), vInput.size() };
	const Eigen::Vector2i BoundaryResolution{ vBoundary.front().size(), vBoundary.size() };
	
	float BoundaryAverageHeight = 0.0f;
	std::size_t NumBoundaryItems = 0;
	for (int Y = 0; Y < BoundaryResolution.y(); Y++)
		for (int X = 0; X < BoundaryResolution.x(); X++)
		{
			auto& Lattice = vBoundary[Y][X];
			if (Lattice.Height(0, 0))
			{
				BoundaryAverageHeight += Lattice.Height(0, 0);
				NumBoundaryItems++;
			}
		}
	BoundaryAverageHeight /= NumBoundaryItems;

	float InputAverageHeight = 0.0f;
	std::size_t NumInputItems = 0;
	for (int Y = 0; Y < InputResolution.y(); Y++)
		for (int X = 0; X < InputResolution.x(); X++)
		{
			auto& Lattice = vInput[Y][X];
			if (Lattice.Height(0, 0))
			{
				InputAverageHeight += Lattice.Height(0, 0);
				NumInputItems++;
			}
		}
	InputAverageHeight /= NumInputItems;

	auto DeltaAverageHeight = BoundaryAverageHeight - InputAverageHeight;
	for (int Y = 0; Y < InputResolution.y(); Y++)
		for (int X = 0; X < InputResolution.x(); X++)
			if (vInput[Y][X].Height(0, 0))
				vInput[Y][X].Height(0, 0) += DeltaAverageHeight;
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__generateNewPointsFromLattices(const Eigen::Vector3f& vPlaneNormal, const Eigen::MatrixXi& vMask, const std::vector<std::vector<SLattice>>& vPlaneLattices, std::vector<pcl::PointXYZRGBNormal>& voNewPoints)
{
	_ASSERTE(!vPlaneLattices.empty());
	Eigen::Vector2i Resolution{ vPlaneLattices.front().size(), vPlaneLattices.size() };

	const float K = 1.0f, B = 0.0f;	//Linear coefficient

	Eigen::Vector3f PointNormal = vPlaneNormal;
	if (PointNormal.z() < 0)
		PointNormal = -PointNormal;

	std::vector<pcl::PointXYZRGBNormal> NewPoints;
	for (int X = 0; X < Resolution.x(); X++)
	{
		for (int Y = 0; Y < Resolution.y(); Y++)
		{
			auto& Lattice = vPlaneLattices[Y][X];
			if (Lattice.Indices.empty())
			{
				Eigen::Vector3f RealPos = Lattice.CenterPos + vPlaneNormal * (K * Lattice.Height(0, 0) + B);	
				pcl::PointXYZRGBNormal TempPoint;
				TempPoint.x = RealPos.x();
				TempPoint.y = RealPos.y();
				TempPoint.z = RealPos.z();
				TempPoint.normal_x = PointNormal.x();
				TempPoint.normal_y = PointNormal.y();
				TempPoint.normal_z = PointNormal.z();
				TempPoint.r = Lattice.Color.x();
				TempPoint.g = Lattice.Color.y();
				TempPoint.b = Lattice.Color.z();
				if (!Lattice.Color.norm())
					int i = 0;
				NewPoints.push_back(TempPoint);
			}
		}
	}
	std::swap(voNewPoints, NewPoints);
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector4f CHoleRepairer::__calculatePlaneByIndices(const std::vector<pcl::index_t>& vIndices, Eigen::Matrix3f& vRotationMatrix)
{
	_ASSERTE(!vIndices.empty());
	pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(auto Index: vIndices)
	{
		auto TempPos = CPointCloudRetouchManager::getInstance()->getScene().getPositionAt(Index);
		Eigen::Vector3f Pos{ TempPos.x(), TempPos.y(), TempPos.z() };
		Pos = vRotationMatrix * Pos;
		pcl::PointXYZ TempPoint;
		TempPoint.x = Pos.x(); TempPoint.y = Pos.y(); TempPoint.z = Pos.z();
		BoundaryCloud->push_back(TempPoint);
	}
	return CPlanarityFeature::fitPlane(BoundaryCloud, 0.2f, { 0.0f, 0.0f, 1.0f });
}

//*****************************************************************
//FUNCTION: 
std::vector<std::size_t> CHoleRepairer::__calcAxisOrder(const Eigen::Vector4f& vPlane)
{
	Eigen::Vector3f PlaneNormal{ vPlane.x(), vPlane.y(), vPlane.z() };
	auto MinAxis = PlaneNormal.maxCoeff();
	std::vector<std::size_t> AxisOrder(3);	//Min is at the end
	if (MinAxis == PlaneNormal.x())
		AxisOrder = { 1, 2, 0 };
	else if (MinAxis == PlaneNormal.y())
		AxisOrder = { 2, 0, 1 };
	else if (MinAxis == PlaneNormal.z())
		AxisOrder = { 0, 1, 2 };
	return AxisOrder;
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__gaussBlurbyHeightMatrix(const Eigen::Matrix<Eigen::Matrix<float, 1, 1>, -1, -1>& vHeightMatrix, std::vector<std::vector<SLattice>>& vPlaneLattices)
{
	Eigen::Vector2i Resolution = { vPlaneLattices.front().size(), vPlaneLattices.size() };
	const int KernelSize = 3;
	const int Delta = KernelSize / 2;
	const float KernelRate[3] = { 0.0947f, 0.1183f, 0.1478f };
	for (int Y = 0; Y < Resolution.y(); Y++)
	{
		for (int X = 0; X < Resolution.x(); X++)
		{
			auto& Lattice = vPlaneLattices[Y][X];
			if (!Lattice.Height(0, 0))
			{
				float Sum = 0;
				for (int i = - Delta; i <= Delta; i++)
					for (int k = - Delta; k <= Delta; k++)
						if (Y + i >= 0 && Y + i < Resolution.y() && X + k >= 0 && X + k < Resolution.x() && vHeightMatrix(Y + i, X + k)(0, 0))
						{
							float Rate;
							if (i * k == 0)
								if (i != k)
									Rate = KernelRate[1];
								else
									Rate = KernelRate[2];
							else
								Rate = KernelRate[0];
							Sum += Rate * vHeightMatrix(Y + i, X + k)(0, 0);
						}

				Lattice.Height(0, 0) = Sum / pow(KernelSize, 2);
			}
		}
	}
}

Eigen::MatrixXi CHoleRepairer::__genMask(const Eigen::Vector2i& vResolution, const std::vector<std::vector<SLattice>>& vPlaneLattices)
{
	Eigen::Vector2i LatticesResolution{ vPlaneLattices.front().size(), vPlaneLattices.size() };
	Eigen::Vector2f ResampleRate{ (float)LatticesResolution.x() / vResolution.x(), (float)LatticesResolution.y() / vResolution.y() };

	_ASSERTE(vResolution.x() <= LatticesResolution.x() && vResolution.y() <= LatticesResolution.y());
	Eigen::MatrixXi Mask(LatticesResolution.y(), LatticesResolution.x());
	for (int Y = 0; Y < Mask.rows(); Y++)
		for (int X = 0; X < Mask.cols(); X++)
			Mask(Y, X) = 1;

	for (int Y = 0; Y < Mask.rows(); Y++)
		for (int X = 0; X < Mask.cols(); X++)
			if (vPlaneLattices[Y][X].Color.norm())
				Mask(Y, X) = 0;
			else
				Mask(Y, X) = 1;

	return Mask;
}

void CHoleRepairer::__outputImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath)
{
	const auto Width = vTexture.cols();
	const auto Height = vTexture.rows();
	const auto BytesPerPixel = 3;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto Offset = (i * Width + k) * BytesPerPixel;
			ResultImage[Offset] = vTexture.coeff(i, k)[0];
			ResultImage[Offset + 1] = vTexture.coeff(i, k)[1];
			ResultImage[Offset + 2] = vTexture.coeff(i, k)[2];
		}

	stbi_write_png(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}

void CHoleRepairer::__outputImage(const Eigen::MatrixXi& vTexture, const std::string& vOutputImagePath)
{
	const auto Width = vTexture.cols();
	const auto Height = vTexture.rows();
	const auto BytesPerPixel = 3;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto Offset = (i * Width + k) * BytesPerPixel;
			auto Color = vTexture.coeff(i, k) == 0 ? 0 : 255;
			ResultImage[Offset] = Color;
			ResultImage[Offset + 1] = Color;
			ResultImage[Offset + 2] = Color;
		}

	stbi_write_png(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}

void CHoleRepairer::__outputImage(const Eigen::Matrix<Eigen::Matrix<float, 1, 1>, -1, -1>& vTexture, const std::string& vOutputImagePath)
{
	const auto Width = vTexture.cols();
	const auto Height = vTexture.rows();
	const auto BytesPerPixel = 3;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];

	float MinHeight = FLT_MAX, MaxHeight = -FLT_MAX;
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto Matrixf = vTexture(i, k);
			auto Height = Matrixf(0, 0);
			if (Height < MinHeight)
				MinHeight = Height;
			if (Height > MaxHeight)
				MaxHeight = Height;
		}

	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto Offset = (i * Width + k) * BytesPerPixel;
			auto Matrixf = vTexture(i, k);
			auto Height = Matrixf(0, 0);
			if (Height > 0)
			{
				ResultImage[Offset] = Height / MaxHeight * 255;
				ResultImage[Offset + 1] = 0;
				ResultImage[Offset + 2] = 0;
			}
			else if (Height < 0)
			{
				ResultImage[Offset] = 0;
				ResultImage[Offset + 1] = Height / MinHeight * 255;
				ResultImage[Offset + 2] = 0;
			}
			else
			{
				ResultImage[Offset] = 255;
				ResultImage[Offset + 1] = 255;
				ResultImage[Offset + 2] = 255;
			}
		}

	stbi_write_png(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__reset()
{
	m_BoundarySet.clear();
	m_Input.clear();
}

void CHoleRepairer::__restoreWorldCoord(SPlaneInfos& vioPlaneInfos, std::vector<std::vector<SLattice>>& vioPlaneLattices, Eigen::Matrix3f& vRotationMatrix)
{
	vioPlaneInfos.Normal = vRotationMatrix.inverse() * vioPlaneInfos.Normal;
	vioPlaneInfos.Normal.normalize();
	
	vioPlaneInfos.PlaneCenter = vRotationMatrix.inverse() * vioPlaneInfos.PlaneCenter;
	for(auto& Row: vioPlaneLattices)
		for(auto& Lattice : Row)
		{
			Lattice.CenterPos = vRotationMatrix.inverse() * Lattice.CenterPos;
		}
}