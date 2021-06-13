#pragma once

#define RECORD_TIME_BEGIN clock_t StartTime, FinishTime;\
StartTime = clock();

#define RECORD_TIME_END(Name) FinishTime = clock();\
std::cout << "\n" << #Name << "花费时间: " << (int)(FinishTime - StartTime) << " ms\n";

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		const std::string CLASSIFIER_BINARY = "Binary";
		const std::string CLASSIFIER_SpatialCluster = "Cluster";
		const std::string CLASSIFIER_MaxVisibilityCluster = "MaxVisibilityCluster";
		const std::string CLASSIFIER_REGION_GROW = "RegionGrow";
		const std::string CLASSIFIER_REGION_GROW_COLOR = "RegionGrow_color";
		const std::string CLASSIFIER_OUTLIER_DETECTION = "OutlierDetection";

		enum class EPointLabel : unsigned char
		{
			DISCARDED,
			KEPT,
			UNWANTED,
			UNDETERMINED,
			FILLED,
		};

		struct SPointLabelChange
		{
			pcl::index_t Index = INT_MAX;
			EPointLabel SrcLabel;
			EPointLabel DstLabel;
			float Confidence = 1.0f;

			friend bool operator < (const SPointLabelChange& vLeft, const SPointLabelChange& vRight) { return vLeft.Index < vRight.Index; }
		};

		struct SBox
		{
			Eigen::Vector3f Min{ FLT_MAX, FLT_MAX, FLT_MAX };
			Eigen::Vector3f Max{ -FLT_MAX, -FLT_MAX, -FLT_MAX };

			inline bool isInBox(float vX, float vY, float vZ)
			{
				if (vX < Min[0] || vY < Min[1] || vZ < Min[2] || vX > Max[0] || vY > Max[1] || vZ > Max[2])
					return false;
				else
					return true;
			}

			void update(const SBox& vBox)
			{
				for (int i = 0; i < 3; ++i)
				{
					if (Min[i] > vBox.Min[i])
						Min[i] = vBox.Min[i];

					if (Max[i] < vBox.Max[i])
						Max[i] = vBox.Max[i];
				}
			}
			void update(float vX, float vY, float vZ)
			{
				if (vX < Min[0])
					Min[0] = vX;
				if (vY < Min[1])
					Min[1] = vY;
				if (vZ < Min[2])
					Min[2] = vZ;
				if (vX > Max[0])
					Max[0] = vX;
				if (vY > Max[1])
					Max[1] = vY;
				if (vZ > Max[2])
					Max[2] = vZ;
			}
			void reset()
			{
				Min = { FLT_MAX, FLT_MAX, FLT_MAX };
				Max = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
			}
		};
	}
}