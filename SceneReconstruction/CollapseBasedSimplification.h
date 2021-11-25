#pragma once
#include "MeshSimplification.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CCollapseBasedSimplification : public IMeshSimplifacation
		{
		public:
			CCollapseBasedSimplification() = default;
			~CCollapseBasedSimplification() = default;

			virtual CMesh simplifyMesh() override;

		private:
			void __toManifold(CMesh& vioMesh);
			std::vector<Eigen::Vector3f> __calcVertexNormal(const CMesh& vMesh);
		};
	}
}

