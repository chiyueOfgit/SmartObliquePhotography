#pragma once
#include "Mesh.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>


namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class IMeshParameterizer : public hiveDesignPattern::IProduct
		{
		public:
			IMeshParameterizer() = default;
			virtual ~IMeshParameterizer() = default;

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh);

			virtual bool execute(Eigen::MatrixXd& voUV) = 0;

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			CMesh m_Mesh;
		};
	}
}