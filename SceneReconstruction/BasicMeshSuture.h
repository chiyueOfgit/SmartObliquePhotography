#pragma once
#include "MeshSuture.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CBasicMeshSuture : public IMeshSuture
		{
		public:
			CBasicMeshSuture() = default;
			~CBasicMeshSuture() = default;

			virtual void sutureMeshes(CMesh& voMeshLHS, CMesh& voMeshRHS) override;
		};
	}
}

