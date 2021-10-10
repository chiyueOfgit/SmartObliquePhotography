#pragma once
#include "MeshSimplification.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CMeshSimplification : public IMeshSimplifacation
		{
		public:
			CMeshSimplification() = default;
			~CMeshSimplification() = default;

			virtual void simplifyMesh() override;

		private:

		};
	}
}

