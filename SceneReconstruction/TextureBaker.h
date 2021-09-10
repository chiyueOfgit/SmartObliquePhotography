#pragma once
#include "Mesh.h"
#include "Image.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class ITextureBaker : public hiveDesignPattern::IProduct
		{
		public:
			ITextureBaker() = default;
			virtual ~ITextureBaker() = default;

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh);
			virtual CImage<std::array<int, 3>> bakeTexture(PointCloud_t::Ptr vPointCloud, const Eigen::Vector2i& vResolution) = 0;

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			CMesh m_Mesh;
		};
	}
}
