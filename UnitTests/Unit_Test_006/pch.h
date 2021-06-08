//
// pch.h
//

#pragma once

#include "framework.h"
#include "gtest/gtest.h"
#include <vector>
#include <string>
#include <map>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "common/Product.h"
#include "common/Factory.h"
#include "common/Singleton.h"
#include "common/CommonInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/EventLoggerInterface.h"