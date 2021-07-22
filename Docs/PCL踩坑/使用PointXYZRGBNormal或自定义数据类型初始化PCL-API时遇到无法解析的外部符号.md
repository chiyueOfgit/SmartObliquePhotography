# ***使用PointXYZRGBNormal或自定义数据类型初始化PCL API时遇到无法解析的外部符号***

## **一. 问题描述**

以 `pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne` 为例  

会报错 `错误LNK2001：无法解析的外部符号`

## **二. 解决方法**

PCL API基于模板，默认情况下会实例化最常见的用例（通常在.cpp文件中完成，例如：normal_3d.cpp）  

`PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointSurfel)(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal))((pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal)))`

PCL库没有初始化PointXYZRGBNormal或自定义的数据类型，所以需要自己做初始化
```C++
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>  // defines the PCL_INSTANTIATE_PRODUCT macro
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp> // make sure to include the .hpp file

PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointXYZRGBNormal))((pcl::Normal)))

pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;
```
