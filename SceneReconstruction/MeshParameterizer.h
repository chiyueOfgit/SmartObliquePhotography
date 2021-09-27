#pragma once
#include "Mesh.h"

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

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			CMesh m_Mesh;
		};
	}
}