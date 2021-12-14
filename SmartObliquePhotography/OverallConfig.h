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

		std::tuple<std::string, std::uint32_t, std::uint32_t> getFirstPointCloudFile() const;    //这里返回值使用std::string而不是const std::string&，是希望通过放回empty std::string表明文件遍历结束
		std::tuple<std::string, std::uint32_t, std::uint32_t> getNextPointCloudFile() const;

	private:
		COverallConfig() = default;

		bool m_IsReady = false;

	friend class hiveDesignPattern::CSingleton<COverallConfig>;
	};
}

