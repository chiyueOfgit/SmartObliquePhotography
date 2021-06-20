#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointLabelSet
		{
		public:
			CPointLabelSet() = default;
			~CPointLabelSet() = default;

			void tagPointLabel(const std::vector<pcl::index_t>& vTargetPointSet, EPointLabel vTargetLabel);
			void init(std::size_t vSize);

		private:
			std::vector<EPointLabel> m_LabelSet;
		};
	}
}

