#pragma once
#include "Command.h"
#include "AutoRetouchCommon.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointCluster;

		class IClutterEraser : public ICommand
		{
		public:
			IClutterEraser();
			virtual ~IClutterEraser();

			void refine(const std::vector<SPointLabelChange>& vPointLabelChangeSet);
			void resetPointLabel(const std::vector<SPointLabelChange>& vPointLabelChangeSet);

		protected
			void _changePointLabel(const std::vector<SPointLabelChange>& vPointLabelChangeSet);

		private:
			std::vector<EPointLabel> m_PointLabelSet;
			std::vector<CPointCluster*> m_PointClusterSet;
		};
	}
}
