#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CBoundaryDetector;

		struct SLattice
		{
			std::vector<int> Indices;
			Eigen::Vector3f CenterPos{ 0.0f, 0.0f, 0.0f };
			Eigen::Vector3i Color{ 0, 0, 0 };
			Eigen::Matrix<float, 1, 1> Height{ 0.0f };
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

			bool init(const hiveConfig::CHiveConfig* vConfig);

			void setHoleRegion(const std::vector<pcl::index_t>& vHoleRegion);
			void setInput(const std::vector<pcl::index_t>& vInputIndices) { m_Input = vInputIndices; }
			void repairHole(std::vector<pcl::PointSurfel>& voNewPoints);

			void repairHoleByBoundaryAndInput(const std::vector<pcl::index_t>& vBoundaryIndices, const std::vector<pcl::index_t>& vInputIndices, std::vector<pcl::PointSurfel>& voNewPoints);

		private:
			void __generatePlaneLattices(const Eigen::Vector4f& vPlane, const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox, const Eigen::Vector2i& vResolution, SPlaneInfos& voPlaneInfos, std::vector<std::vector<SLattice>>& voPlaneLattices);
			void __projectPoints2PlaneLattices(const std::vector<pcl::index_t>& vIndices, const SPlaneInfos& vPlaneInfos, std::vector<std::vector<SLattice>>& vioPlaneLattices);
			void __fillLatticesOriginInfos(const Eigen::Vector3f& vNormal, std::vector<std::vector<SLattice>>& vioPlaneLattices);
			void __generateNewPointsFromLattices(const Eigen::Vector4f& vPlane, const std::vector<std::vector<SLattice>>& vPlaneLattices, std::vector<pcl::PointSurfel>& voNewPoints);

			Eigen::Vector4f __calculatePlaneByIndices(const std::vector<pcl::index_t>& vIndices);
			std::pair<Eigen::Vector3f, Eigen::Vector3f> __calculateBoundingBoxByIndices(const std::vector<pcl::index_t>& vIndices);
			Eigen::MatrixXi __genMask()
			{
				Eigen::MatrixXi Mask(m_Resolution.y(), m_Resolution.x());
				for (int Y = 0; Y < Mask.rows(); Y++)
					for (int X = 0; X < Mask.cols(); X++)
						Mask(Y, X) = 1;
				return Mask;
			}

			template<class T>
			std::vector<std::vector<T>> __matrix2Vector(const Eigen::Matrix<T, -1, -1>& vMatrix)
			{
				std::vector<std::vector<T>> TempVector(vMatrix.rows(), std::vector<T>(vMatrix.cols()));
				for (int Y = 0; Y < vMatrix.rows(); Y++)
				{
					for (int X = 0; X < vMatrix.cols(); X++)
					{
						TempVector[Y][X] = vMatrix(Y, X);
					}
				}
				return TempVector;
			}
			template<class T>
			Eigen::Matrix<T, -1, -1> __vector2Matrix(const std::vector<std::vector<T>>& vVector)
			{
				Eigen::Matrix<T, -1, -1> TempMatrix;
				TempMatrix.resize(vVector.size(), vVector.front().size());
				for (int Y = 0; Y < TempMatrix.rows(); Y++)
				{
					for (int X = 0; X < TempMatrix.cols(); X++)
					{
						TempMatrix(Y, X) = vVector[Y][X];
					}
				}
				return TempMatrix;
			}
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

			void __reset();

			const Eigen::Vector2i m_Resolution{ 32, 32 };

			std::vector<std::vector<pcl::index_t>> m_BoundarySet;
			std::vector<pcl::index_t> m_Input;

			CBoundaryDetector* m_pBoundaryDetector = nullptr;
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			const hiveConfig::CHiveConfig* m_pTextureConfig = nullptr;
		};
	}
}
