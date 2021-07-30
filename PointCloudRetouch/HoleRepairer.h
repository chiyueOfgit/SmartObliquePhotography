#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		struct SLattice
		{
			std::vector<int> Indices;
			Eigen::Vector3f CenterPos{ 0.0f, 0.0f, 0.0f };
			Eigen::Vector3i Color{0, 0, 0};
			float Height = 0.0f;
		};

		struct SPlaneInfos
		{
			Eigen::Vector3f Normal;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> BoundingBox;
			Eigen::Vector3f PlaneCenter;
			Eigen::Vector2f LatticeSize;
			std::vector<std::size_t> AxisOrder;
		};

		class CHoleRepairer
		{
		public:
			CHoleRepairer() = default;
			~CHoleRepairer() = default;

			void repairHoleByBoundaryAndInput(const std::vector<pcl::index_t>& vBoundaryIndices, const std::vector<pcl::index_t>& vInputIndices, std::vector<pcl::PointSurfel>& voNewPoints, const hiveConfig::CHiveConfig* vConfig);

		private:
			void __generatePlaneLattices(const Eigen::Vector4f& vPlane, const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox, const Eigen::Vector2i& vResolution, SPlaneInfos& voPlaneInfos, std::vector<std::vector<SLattice>>& voPlaneLattices);
			
			void __projectPoints2PlaneLattices(const std::vector<pcl::index_t>& vIndices, const SPlaneInfos& vPlaneInfos, std::vector<std::vector<SLattice>>& vioPlaneLattices);

			void __generateNewPointsFromLattices(const Eigen::Vector4f& vPlane, const std::vector<std::vector<SLattice>>& vPlaneLattices, std::vector<pcl::PointSurfel>& voNewPoints);
			
			Eigen::Vector4f __calculatePlaneByIndices(const std::vector<pcl::index_t>& vIndices);
			std::pair<Eigen::Vector3f, Eigen::Vector3f> __calculateBoundingBoxByIndices(const std::vector<pcl::index_t>& vIndices);

			template<class T>
			std::vector<std::vector<T>> __extractItemFromLattices(const std::vector<std::vector<SLattice>>& vLattices, int vOffset)
			{
				_ASSERTE(!vLattices.empty());
				Eigen::Vector2i Resolution{ vLattices.front().size(), vLattices.size() };
				std::vector<std::vector<T>> Items(Resolution.y(), std::vector<T>(Resolution.x()));

				for (int Y = 0; Y < Resolution.y(); Y++)
				{
					for (int X = 0; X < Resolution.x(); X++)
					{
						const SLattice& Lattice = vLattices[Y][X];
						void* Ptr = (bool*)(&Lattice) + vOffset;
						T* Item = static_cast<T*>(Ptr);
						Items[Y][X] = *Item;
					}
				}

				return Items;
			}
			template<class T>
			void __fillLatticesByItems(const std::vector<std::vector<T>>& vItems, std::vector<std::vector<SLattice>>& vLattices, int vOffset)
			{
				_ASSERTE(!vItems.empty());

				Eigen::Vector2i Resolution{ vLattices.front().size(), vLattices.size() };
				_ASSERTE(vItems.front().size() == Resolution.x() && vItems.size() == Resolution.y());

				for (int Y = 0; Y < Resolution.y(); Y++)
				{
					for (int X = 0; X < Resolution.x(); X++)
					{
						SLattice& Lattice = vLattices[Y][X];
						void* LatticePtr = (bool*)(&Lattice) + vOffset;
						T* ItemLattice = static_cast<T*>(LatticePtr);
						*ItemLattice = vItems[Y][X];
					}
				}
			}

		};
	}
}
