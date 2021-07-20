// pch.h: 这是预编译标头文件。
// 下方列出的文件仅编译一次，提高了将来生成的生成性能。
// 这还将影响 IntelliSense 性能，包括代码完成和许多代码浏览功能。
// 但是，如果此处列出的文件中的任何一个在生成之间有更新，它们全部都将被重新编译。
// 请勿在此处添加要频繁更新的文件，这将使得性能优势无效。
#pragma once

#ifndef PCH_H
#define PCH_H

// 添加要在此处预编译的标头
#include "framework.h"

#include <vector>
#include <string>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "common/Product.h"
#include "common/Singleton.h"
#include "common/CommonMicro.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/HiveConfig.h"
#include "common/ConfigInterface.h"
#include "common/UtilityInterface.h"
#include "common/FileSystem.h"


using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;

#endif //PCH_H

