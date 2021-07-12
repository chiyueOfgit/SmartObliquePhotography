#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointClusterExpander;
		class CPointCluster;

		class CRetouchTask
		{
		public:
			CRetouchTask() = default;
			~CRetouchTask() = default;

			bool init(const hiveConfig::CHiveConfig* vConfig);
			bool execute(const CPointCluster *vUserSpecifiedCluster) const;

			void dumpTaskMarkedPoints(std::vector<pcl::index_t>& voMarkedPoints) const;

			const hiveConfig::CHiveConfig* getClusterConfig() const { _ASSERTE(m_pClusterConfig); return m_pClusterConfig; }

#ifdef _UNIT_TEST
			const auto getExpander() const { return m_pPointClusterExpander; }
#endif

		private:
			CPointClusterExpander *m_pPointClusterExpander = nullptr;
			const hiveConfig::CHiveConfig *m_pConfig = nullptr;
			const hiveConfig::CHiveConfig* m_pClusterConfig = nullptr;
		};
	}
}

