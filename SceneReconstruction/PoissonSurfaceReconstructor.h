#pragma once
#include "SurfaceReconstructor.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CPoissonSurfaceReconstructor : public ISurfaceReconstructor
		{
		public:
			CPoissonSurfaceReconstructor() = default;
			~CPoissonSurfaceReconstructor() = default;

			virtual void constructSurface(CMesh& voMesh) override;
			
		private:
			std::vector<Eigen::Vector3f> __calcVertexNormal(const CMesh& vMesh);
			void __executePostProcessing(CMesh& vioMesh);
		};
	}
}
