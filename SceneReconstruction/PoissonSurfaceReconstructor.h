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

			virtual void constructSurface(pcl::PolygonMesh& voMesh) override;
			
		private:
		};
	}
}
