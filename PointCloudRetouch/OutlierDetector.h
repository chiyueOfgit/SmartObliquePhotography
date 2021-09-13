#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class COutlierDetector : public IPointClassifier
		{
		public:
			COutlierDetector() = default;
			~COutlierDetector() = default;

			virtual void runV(pcl::Indices& vInputIndices, EPointLabel vTargetLabel, const float SEARCH_RADIUS, const int  MIN_NEIGHBORS_IN_RADIUS, const bool POINT_FILTER_CONDITION);
#ifdef _UNIT_TEST
			double getRunTime(){ return m_RunTime; }
#endif // _UNIT_TEST

		private:
#ifdef _UNIT_TEST
			double m_RunTime = 0;
#endif // _UNIT_TEST


		};
	}
}
