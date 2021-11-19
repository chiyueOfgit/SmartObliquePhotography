#include "pch.h"
#include "ElevationMapGenerator.h"
#include "AutoHoleRepairer.h"
#include "NewMipmapGenerator.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

void CAutoHoleRepairer::execute(const Eigen::Vector2i& vResolution, std::vector<pcl::index_t>& vPointCloudIndices, std::vector<pcl::PointXYZRGBNormal>& voNewPointSet)
{
	CElevationMapGenerator HoleMapGenerator;
	CImage<float> ElevationMap;
	HoleMapGenerator.execute(vResolution, vPointCloudIndices);
    HoleMapGenerator.dumpElevationMap(ElevationMap);
	saveTexture("New.png", ElevationMap, false);
	std::vector<Eigen::Vector2i> HoleSet;
	__repairImageByMipmap(ElevationMap, HoleSet);
	__executeMeanFilter(ElevationMap, 5);
	if(m_PointDistributionSet.empty())
	{
		std::vector<pcl::index_t> SceneIndices;
		for (int i = 0; i < CPointCloudRetouchManager::getInstance()->getScene().getNumPoint(); i++)
			SceneIndices.push_back(i);
		HoleMapGenerator.generateDistributionSet(vResolution, SceneIndices);
		HoleMapGenerator.dumpPointDistributionSet(m_PointDistributionSet);
	}
	voNewPointSet = __generateNewPoint(HoleSet, ElevationMap);
}

void CAutoHoleRepairer::setPointDistributionSet(std::vector<std::vector<std::vector<pcl::index_t>>>& vPointDistributionSet)
{
	m_PointDistributionSet = vPointDistributionSet;
}

void CAutoHoleRepairer::__repairImageByScan(CImage<float>& vioHoleImage, std::vector<Eigen::Vector2i>& voHoleSet)
{
	auto Width = vioHoleImage.getWidth();
	auto Height = vioHoleImage.getHeight();

	for (int i = 0; i < Width; i++)
	{
		for (int k = 0; k < Height; k++)
		{
			if (vioHoleImage.getColor(k, i) == 0)
			{
				voHoleSet.push_back({ i,k });
			}
		}
	}
	
	int Directions[8][2] = { {-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0} };
	for (int i = 0; i < Width; i++)
	{
		for (int k = 0; k < Height; k++)
		{
			Eigen::Vector2i CurrentPixel{ i,k };
			if (vioHoleImage.getColor(CurrentPixel.y(), CurrentPixel.x()) != 0)
				continue;
			int NeighborNum = 0;
			float ColorSum = 0;
			for (auto& Direction: Directions)
			{
				Eigen::Vector2i NeighborPixel;
				NeighborPixel.x() = CurrentPixel.x() + Direction[0];
				NeighborPixel.y() = CurrentPixel.y() + Direction[1];
				if (NeighborPixel.x() < 0 || NeighborPixel.y() < 0 || NeighborPixel.x() > (Width - 1) || (NeighborPixel.y() > Height - 1))
					continue;
				if (vioHoleImage.getColor(NeighborPixel.y(), NeighborPixel.x()) != 0)
				{
					ColorSum += vioHoleImage.getColor(NeighborPixel.y(), NeighborPixel.x());
					NeighborNum++;
				}
			}
			ColorSum /= NeighborNum;
			vioHoleImage.fetchColor(CurrentPixel.y(), CurrentPixel.x()) = ColorSum;
		}
	}
}

void CAutoHoleRepairer::__repairImageByRound2Center(CImage<float>& vioHoleImage, std::vector<Eigen::Vector2i>& voHoleSet)
{
	auto Width = vioHoleImage.getWidth();
	auto Height = vioHoleImage.getHeight();
	std::vector<Eigen::Vector2i> TempHoleSet;
	std::vector<std::vector<bool>> Flag(Height, std::vector<bool>(Width, false));
	auto TempImage = vioHoleImage;

	int Directions[8][2] = { {-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0} };
	for (int i = 0; i < Width; i++)
	{
		for (int k = 0; k < Height; k++)
		{
			if (vioHoleImage.getColor(k, i) == 0)
			{
				voHoleSet.push_back({ i,k });
				Flag[k][i] = true;
			}
		}
	}
	while (true)
	{
		TempHoleSet.clear();
		for (auto& HolePos : voHoleSet)
		{
			if (!Flag[HolePos.y()][HolePos.x()])
				continue;
			int NeighborNum = 0;
			float ColorSum = 0;
			for (auto& Direction : Directions)
			{
				Eigen::Vector2i NeighborPixel;

				NeighborPixel.x() = HolePos.x() + Direction[0];
				NeighborPixel.y() = HolePos.y() + Direction[1];
				if (NeighborPixel.x() < 0 || NeighborPixel.y() < 0 || NeighborPixel.x() > (Width - 1) || (NeighborPixel.y() > Height - 1))
					continue;
				if (vioHoleImage.getColor(NeighborPixel.y(), NeighborPixel.x()) != 0.0f)
				{
					ColorSum += vioHoleImage.getColor(NeighborPixel.y(), NeighborPixel.x());
					NeighborNum++;
				}
			}
			if (NeighborNum)
			{
				ColorSum /= NeighborNum;
				TempImage.fetchColor(HolePos.y(), HolePos.x()) = ColorSum;
				Flag[HolePos.y()][HolePos.x()] = false;
				TempHoleSet.push_back(HolePos);
			}
		}
		vioHoleImage = TempImage;
		if (TempHoleSet.empty())
			break;
	}
}

void CAutoHoleRepairer::__repairImageByMipmap(CImage<float>& vioHoleImage, std::vector<Eigen::Vector2i>& voHoleSet)
{
	auto Width = vioHoleImage.getWidth();
	auto Height = vioHoleImage.getHeight();
	std::vector<Eigen::Vector2i> TempHoleSet;
	
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Texture(Height, Width);
	
	for (int i = 0; i < Width; i++)
	{
		for (int k = 0; k < Height; k++)
		{
			if (vioHoleImage.getColor(k, i) == 0)
			{
				voHoleSet.push_back({ i,k });
				Texture(k, i) = -1;
			}
			else
			{
				Texture(k, i) = vioHoleImage.fetchColor(k, i);
			}
		}
	}
	//TODO:Size大小由分辨率确定
	int Size = 10;
	HiveTextureSynthesizer::CNewMipmapGenerator<float> MipmapGenerator;
	auto MipmapSet = MipmapGenerator.computeMipmapPyramid(Texture, Size);

	for(int i = 0; i < Size - 1; i++)
	{
		auto CurrentTexture = MipmapSet[i];
		auto& NextTexture = MipmapSet[i + 1];
		for(int j = 0; j < CurrentTexture.cols(); j++)
			for(int k = 0; k< CurrentTexture.rows(); k++)
			{
				auto CurrentColor = CurrentTexture.coeff(k, j);
				float SumColor = 0.0f;
				std::vector<Eigen::Vector2i> CorrespondingCoord{ {2 * k,2 * j}, {2 * k + 1,2 * j}, {2 * k,2 * j + 1},{2 * k + 1,2 * j + 1} };
				int BlackNum = CorrespondingCoord.size();
				for(auto& Coord: CorrespondingCoord)
				{
					if(NextTexture.coeff(Coord.x(), Coord.y()) > 0)
					{
						SumColor += NextTexture.coeff(Coord.x(), Coord.y());
						BlackNum--;
					}
				}
				if(BlackNum == 0)
					continue;
				if(BlackNum == CorrespondingCoord.size())
					for (auto& Coord : CorrespondingCoord)
							NextTexture(Coord.x(), Coord.y()) = CurrentColor;
				else
				{
					auto NextColor = (CurrentColor * 4 - SumColor) / BlackNum;
					for (auto& Coord : CorrespondingCoord)
						if (NextTexture.coeff(Coord.x(), Coord.y()) <= 0)
							NextTexture(Coord.x(), Coord.y()) = NextColor;
				}
			}
	}
	for (int i = 0; i < Width; i++)
	{
		for (int k = 0; k < Height; k++)
		{
			vioHoleImage.fetchColor(k, i) = MipmapSet[Size - 1].coeff(k, i);
		}
	}
}

std::vector<pcl::PointXYZRGBNormal> CAutoHoleRepairer::__generateNewPoint(const std::vector<Eigen::Vector2i>& vHoleSet, const CImage<float>& vWithoutHoleMap)
{
	std::vector<pcl::PointXYZRGBNormal> NewPointSet;
	auto pManager = CPointCloudRetouchManager::getInstance();
	auto Box = pManager->getScene().getBoundingBox(std::vector<pcl::index_t>());

	for (auto& HolePos : vHoleSet)
	{
		auto Indices = m_PointDistributionSet[HolePos.y()][HolePos.x()];
		for (auto Index : Indices)
		{
			pcl::PointXYZRGBNormal NewPoint;
			NewPoint.x = pManager->getScene().getPositionAt(Index).x();
			NewPoint.y = pManager->getScene().getPositionAt(Index).y();
			NewPoint.z = (vWithoutHoleMap.getColor(HolePos.y(), HolePos.x())) / 255.0f * (Box.second - Box.first).z() + Box.first.z();
			NewPoint.normal_x = pManager->getScene().getNormalAt(Index).x();
			NewPoint.normal_y = pManager->getScene().getNormalAt(Index).y();
			NewPoint.normal_z = pManager->getScene().getNormalAt(Index).z();
			NewPoint.r = pManager->getScene().getColorAt(Index).x();
			NewPoint.g = pManager->getScene().getColorAt(Index).y();
			NewPoint.b = pManager->getScene().getColorAt(Index).z();
			NewPointSet.push_back(NewPoint);
		}
	}
	return NewPointSet;
}

void CAutoHoleRepairer::__executeMeanFilter(CImage<float>& vioWithoutHoleImage, int vKernel)
{
	auto Width = vioWithoutHoleImage.getWidth();
	auto Height = vioWithoutHoleImage.getHeight();
	auto Half = vKernel / 2;
	auto TempImage = vioWithoutHoleImage;
	
	for (int i = 0; i < Width; i++)
	{
		for (int k = 0; k < Height; k++)
		{
			Eigen::Vector2i CurrentPixel{ i,k };
			int NeighborNum = 0;
			float ColorSum = 0;
			for (auto RowOffset = - Half; RowOffset <= Half; RowOffset++)
			{
				for(auto ColOffset = - Half; ColOffset <= Half; ColOffset++ )
				{
					Eigen::Vector2i NeighborPixel;
					NeighborPixel.x() = CurrentPixel.x() + RowOffset;
					NeighborPixel.y() = CurrentPixel.y() + ColOffset;
					if (NeighborPixel.x() < 0 || NeighborPixel.y() < 0 || NeighborPixel.x() > (Width - 1) || (NeighborPixel.y() > Height - 1))
						continue;
					ColorSum += vioWithoutHoleImage.getColor(NeighborPixel.y(), NeighborPixel.x());
					NeighborNum++;
				}
			}
			ColorSum /= NeighborNum;
			TempImage.fetchColor(CurrentPixel.y(), CurrentPixel.x()) = ColorSum;
		}
	}
	vioWithoutHoleImage = TempImage;
}

void CAutoHoleRepairer::saveTexture(const std::string& vPath, const CImage<float>& vTexture, bool vIsReverse)
{
	const auto Width = vTexture.getWidth();
	const auto Height = vTexture.getHeight();
	const auto BytesPerPixel = 1;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto I = i;
			if (vIsReverse)
				I = Height - 1 - I;
			auto Offset = (I * Width + k) * BytesPerPixel;
			ResultImage[Offset] = vTexture.getColor(i, k);
		}

	stbi_write_png(vPath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}