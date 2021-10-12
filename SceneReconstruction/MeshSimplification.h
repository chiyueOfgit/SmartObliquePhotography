#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class IMeshSimplifacation : public hiveDesignPattern::IProduct
		{
		public:
			IMeshSimplifacation() = default;
			virtual ~IMeshSimplifacation() = default;

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh);
			virtual CMesh simplifyMesh() = 0;

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			CMesh m_Mesh;
		};
	}
}
