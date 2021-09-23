#include "pch.h"
#include "NormalComplexityVisualization.h"
#include "PointCloudVisualizer.h"
#include <omp.h>

using namespace hiveObliquePhotography::Feature;

CNormalComplexityVisualization::CNormalComplexityVisualization()
{

}

CNormalComplexityVisualization::~CNormalComplexityVisualization()
{
	
}

void CNormalComplexityVisualization::init(PointCloud_t::Ptr vPointCloud)
{
	_ASSERT(vPointCloud);
	m_pCloud = vPointCloud;
}

void CNormalComplexityVisualization::run()
{
	double Radius = 1.5;
	pcl::PointCloud<pcl::PointNormal>::Ptr pNormalCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*m_pCloud, *pNormalCloud);
	
	pcl::search::Search<pcl::PointNormal>::Ptr Tree;
	if (m_pCloud->isOrganized())
	{
		Tree = std::make_shared<pcl::search::OrganizedNeighbor<pcl::PointNormal>>();
	}
	else
	{
		Tree = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
	}

	Tree->setInputCloud(pNormalCloud);
	PointCloud_t::Ptr pFeatureCloud(new PointCloud_t);
	pFeatureCloud->resize(pNormalCloud->size());

#pragma omp parallel for
	for (int i = 0; i < pNormalCloud->size(); i++)
	{
		pcl::Indices Neighborhood;
		std::vector<float> Distance;
		Tree->radiusSearch(i, Radius, Neighborhood, Distance);

		double MinDistance = DBL_MAX;
		double MaxDistance = -DBL_MAX;
		double MeanDistance = 0.0f;

		Eigen::Vector3f Normal = pNormalCloud->at(i).getNormalVector3fMap();

		for (auto& NeighborIndex : Neighborhood)
		{
			const double Temp = pNormalCloud->at(NeighborIndex).getVector3fMap().dot(Normal);
			MeanDistance += Temp;
			if (MinDistance > Temp)
				MinDistance = Temp;
			if (MaxDistance < Temp)
				MaxDistance = Temp;
		}

		MeanDistance /= Neighborhood.size();

		double Complexity = std::min(abs(MinDistance - MeanDistance), abs(MaxDistance - MeanDistance));
		
		pFeatureCloud->at(i).x = pNormalCloud->at(i).x;
		pFeatureCloud->at(i).y = pNormalCloud->at(i).y;
		pFeatureCloud->at(i).z = pNormalCloud->at(i).z;
		
		Complexity *= 255;
		Complexity /= Radius;

		pFeatureCloud->at(i).r = pFeatureCloud->at(i).g = pFeatureCloud->at(i).b = static_cast<uint8_t>(Complexity);
	}

	for (int i = 0; i < pFeatureCloud->size(); i++)
	{
		Visualization::CPointCloudVisualizer::getInstance()->addUserColoredPoints({ i }, { pFeatureCloud->at(i).r , pFeatureCloud->at(i).g, pFeatureCloud->at(i).b});
	}
}