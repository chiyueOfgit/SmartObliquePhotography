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
			void __fixTextureColorAndHeight(std::vector<std::vector<SLattice>>& vioPlaneLattices, int vKernelSize);
			void __generateNewPointsFromLattices(const Eigen::Vector4f& vPlane, const Eigen::MatrixXi& vMask, const std::vector<std::vector<SLattice>>& vPlaneLattices, std::vector<pcl::PointSurfel>& voNewPoints);

			Eigen::Vector4f __calculatePlaneByIndices(const std::vector<pcl::index_t>& vIndices);
			std::pair<Eigen::Vector3f, Eigen::Vector3f> __calculateBoundingBoxByIndices(const std::vector<pcl::index_t>& vIndices);
			float __calcMeanPointsPerLattice(const std::vector<std::vector<SLattice>>& vPlaneLattices);
			Eigen::MatrixXi __genMask(const Eigen::Vector2i& vResolution, const std::vector<std::vector<SLattice>>& vPlaneLattices);

			template<class T>
			Eigen::Matrix<T, -1, -1> __extractMatrixFromLattices(const std::vector<std::vector<SLattice>>& vLattices, int vOffset)
			{
				_ASSERTE(!vLattices.empty());
				Eigen::Vector2i Resolution{ vLattices.front().size(), vLattices.size() };
				Eigen::Matrix<T, -1, -1> Matrix;
				Matrix.resize(Resolution.y(), Resolution.x());

				for (int Y = 0; Y < Resolution.y(); Y++)
				{
					for (int X = 0; X < Resolution.x(); X++)
					{
						const SLattice& Lattice = vLattices[Y][X];
						void* Ptr = (bool*)(&Lattice) + vOffset;
						T* Item = static_cast<T*>(Ptr);
						Matrix(Y, X) = *Item;
					}
				}

				return Matrix;
			}
			template<class T>
			void __fillLatticesByMatrix(const Eigen::Matrix<T, -1, -1>& vMatrix, std::vector<std::vector<SLattice>>& vLattices, int vOffset)
			{
				Eigen::Vector2i Resolution{ vLattices.front().size(), vLattices.size() };
				_ASSERTE(vMatrix.cols() == Resolution.x() && vMatrix.rows() == Resolution.y());

				for (int Y = 0; Y < Resolution.y(); Y++)
				{
					for (int X = 0; X < Resolution.x(); X++)
					{
						SLattice& Lattice = vLattices[Y][X];
						void* LatticePtr = (bool*)(&Lattice) + vOffset;
						T* ItemInLattice = static_cast<T*>(LatticePtr);
						*ItemInLattice = vMatrix(Y, X);
					}
				}
			}

			void __outputImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath);
			void __outputImage(const Eigen::MatrixXi& vTexture, const std::string& vOutputImagePath);
			void __outputImage(const Eigen::Matrix<Eigen::Matrix<float, 1, 1>, -1, -1>& vTexture, const std::string& vOutputImagePath);
			void __reset();

			std::vector<std::vector<pcl::index_t>> m_BoundarySet;
			std::vector<pcl::index_t> m_Input;

			CBoundaryDetector* m_pBoundaryDetector = nullptr;
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			const hiveConfig::CHiveConfig* m_pTextureConfig = nullptr;
		};
	}
}
