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

			virtual void runV(pcl::Indices& vInputIndices, EPointLabel vTargetLabel, float vSearchRadius, int vMinNeighbors, bool vCondition);
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
