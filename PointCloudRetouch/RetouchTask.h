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
			bool execute(const CPointCluster *vUserSpecifiedCluster);

			const hiveConfig::CHiveConfig* getClusterConfig() const;

		private:
			CPointClusterExpander *m_pPointClusterExpander = nullptr;
			const hiveConfig::CHiveConfig* m_pConfig;
		};
	}
}

