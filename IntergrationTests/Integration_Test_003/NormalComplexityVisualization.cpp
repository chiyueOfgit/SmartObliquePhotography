#include "pch.h"
#include "NormalComplexityVisualization.h"
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

}