//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include <deque>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "common/CommonInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/EventLoggerInterface.h"

#include "../../ObliquePhotographyData/ObliquePhotographyDataCommon.h"

using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
