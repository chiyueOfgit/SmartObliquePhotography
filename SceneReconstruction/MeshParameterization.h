#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class IMeshParameterization : public hiveDesignPattern::IProduct
		{
		public:
			IMeshParameterization() = default;
			virtual ~IMeshParameterization() = default;

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh);

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			CMesh m_Mesh;
		};
	}
}