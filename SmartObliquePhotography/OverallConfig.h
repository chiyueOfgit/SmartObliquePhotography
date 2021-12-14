#pragma once
#include "common/Singleton.h"
#include "common/HiveConfig.h"

namespace hiveObliquePhotography
{
	class COverallConfig : public hiveDesignPattern::CSingleton<COverallConfig>, public hiveConfig::CHiveConfig
	{
	public:
		~COverallConfig() = default;

		[[nodiscard]] bool generateSceneInfo();

		bool isReady() const { return m_IsReady; }

		std::tuple<std::string, std::uint32_t, std::uint32_t> getFirstPointCloudFile() const;    //���ﷵ��ֵʹ��std::string������const std::string&����ϣ��ͨ���Ż�empty std::string�����ļ���������
		std::tuple<std::string, std::uint32_t, std::uint32_t> getNextPointCloudFile() const;

	private:
		COverallConfig() = default;

		bool m_IsReady = false;

	friend class hiveDesignPattern::CSingleton<COverallConfig>;
	};
}

