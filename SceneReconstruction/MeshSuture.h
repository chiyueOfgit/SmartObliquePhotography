#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class IMeshSuture : public hiveDesignPattern::IProduct
		{
		public:
			IMeshSuture() = default;
			~IMeshSuture() override = default;

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vLhs, const CMesh& vRhs);
			virtual void sutureMeshesV() = 0;

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			CMesh m_LhsMesh, m_RhsMesh;
		};
	}
}