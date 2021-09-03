//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include <cstdint>
#include <deque>
#include <vector>
#include <string>
#include <set>
#include <ctime>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "common/Product.h"
#include "common/Singleton.h"
#include "common/CommonMicro.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/HiveConfig.h"
#include "common/ConfigInterface.h"
#include "common/UtilityInterface.h"
#include "VisualizationCommon.h"

#include "PointCloudRetouchCommon.h"

using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
