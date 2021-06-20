#include "pch.h"
#include "PointCloudRetouchInterface.h"

//主要围绕CPointCloudRetouchManager::init()进行测试
//注意不要直接一来就直接测试这个函数，看看这个函数的实现，目前在其内部调用了其他几个类的公有函数，要分别针对这些公有函数
//进行测试，最后才是测试CPointCloudRetouchManager::init()

TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}