#pragma once
#include "PointCloudAutoRetouchScene.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster
		{
		public:
			IPointCluster() = default;
			IPointCluster(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel) :
				m_PointIndices(vPointIndices), m_Label(vLabel)
			{
				auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();

				SBox AABB;
				for (auto Index : *m_PointIndices)
					AABB.update((*pCloud)[Index].x, (*pCloud)[Index].y, (*pCloud)[Index].z);
				setClusterAABB(AABB);
			}
			virtual ~IPointCluster() = default;

			EPointLabel getClusterLabel() const { return m_Label; }

			void setClusterAABB(const SBox& vAABB) { m_AABB = vAABB; }
			const SBox& getClusterAABB() const { return m_AABB; }

			const pcl::IndicesPtr& getClusterIndices() const { return m_PointIndices; }
			
			virtual double computeDistanceV(pcl::index_t vPointIndex) const = 0;

		private:
			SBox m_AABB;
			pcl::IndicesPtr m_PointIndices;

			EPointLabel m_Label = EPointLabel::UNDETERMINED;
		};
	}
}
