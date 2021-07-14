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
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/common/common.h"

#include "common/Product.h"
#include "common/Singleton.h"
#include "common/CommonMicro.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/HiveConfig.h"
#include "common/ConfigInterface.h"

#include "PointCloudRetouchCommon.h"

using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
