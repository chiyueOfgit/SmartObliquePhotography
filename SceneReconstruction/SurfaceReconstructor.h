#pragma once

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class ISurfaceReconstructor : public hiveDesignPattern::IProduct
		{
		public:
			ISurfaceReconstructor() = default;
			virtual ~ISurfaceReconstructor() = default;

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, PointCloud_t::Ptr vPointCloudScene);
			virtual void constructSurface(pcl::PolygonMesh& voMesh) = 0;

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			pcl::PointCloud<pcl::PointNormal>::Ptr m_pSceneCloud = nullptr;
		};
	}
}