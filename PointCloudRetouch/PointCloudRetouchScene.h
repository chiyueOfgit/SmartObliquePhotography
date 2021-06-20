#pragma once
#include "PointClusterSet.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class INeighborhoodBuilder;

		class CPointCloudRetouchScene 
		{
		public:
			CPointCloudRetouchScene();
			~CPointCloudRetouchScene();

			void init(PointCloud_t::Ptr vPointCloudScene);
			void reset(std::uint32_t vTimestamp);

#ifdef _UNIT_TEST
			const auto getPointCloudScene() const { return m_pPointCloudScene; }
			const auto getGlobalKdTree() const { return m_pGlobalKdTree; }
#endif // _UNIT_TEST

			std::size_t getNumPoint() const { _ASSERTE(m_pPointCloudScene); return m_pPointCloudScene->size(); }	
			const auto& getPointCloudScene() const { return m_pPointCloudScene; }
		private:
			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
			INeighborhoodBuilder* m_pNeighborhoodBuilder = nullptr;
		};
	}
}


