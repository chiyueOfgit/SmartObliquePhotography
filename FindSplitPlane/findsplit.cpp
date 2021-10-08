#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <vtkAutoInit.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>


void aabb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)//点云AABB包围盒
{
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;

	pcl::PointXYZ min_point_AABB;//AABB包围盒
	pcl::PointXYZ max_point_AABB;

	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

	////绘制AABB包围盒
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);
	////viewer->addCoordinateSystem(1.0);

	//pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud);//设置随机颜色
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, RandomColor, "points");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");

	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

	//pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	//pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	//pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	//pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	//viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	//viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	//viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//}
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr vocloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("../UnitTests/TestData/scu/slice 3 - Cloud.pcd", *vocloud);

	aabb(vocloud);
}
