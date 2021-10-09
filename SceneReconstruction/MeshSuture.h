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
			virtual ~IMeshSuture() = default;

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMeshLHS, const CMesh& vMeshRHS);
			virtual void sutureMeshes() = 0;

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			CMesh m_MeshLHS, m_MeshRHS;
		};
	}
}