#pragma once
#include "common/HiveConfig.h"
#include "common/Singleton.h"
#include "ConfigCommon.h"

class CSpatialClusterConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CSpatialClusterConfig>
{
public:
	virtual ~CSpatialClusterConfig() = default;

private:
	CSpatialClusterConfig() { __defineAttributesV(); }

	void __defineAttributesV() override
	{
		_defineAttribute(KEY_WORDS::RESOLUTION, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
		_defineAttribute(KEY_WORDS::CLUSTERTOLERANCE, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
		_defineAttribute(KEY_WORDS::MINCLUSTERSIZE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
		_defineAttribute(KEY_WORDS::MAXCLUSTERSIZE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
	}
	friend class hiveDesignPattern::CSingleton<CSpatialClusterConfig>;
};