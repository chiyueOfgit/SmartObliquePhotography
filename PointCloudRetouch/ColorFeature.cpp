#include "pch.h"
#include "ColorFeature.h"
#include "PointCloudRetouchManager.h"

#define gamma(x) (x > 0.04045 ? pow((x + 0.055f) / 1.055f, 2.4f) : x / 12.92)
#define deg2Rad(deg) (deg * (M_PI / 180.0))
#define rad2Deg(rad) ((180.0 / M_PI) * rad)

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CColorFeature, KEYWORD::COLOR_FEATURE)

//*****************************************************************
//FUNCTION: 
double CColorFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	_ASSERTE(m_pConfig);
	m_ColorThreshold = *m_pConfig->getAttribute<float>("COLOR_THRESHOLD");
	m_MaxNumMainColors = *m_pConfig->getAttribute<int>("NUM_MAIN_COLORS");
    m_MinReduceRatio = *m_pConfig->getAttribute<float>("MIN_REDUCE_RATIO");
	
	if (vDeterminantPointSet.empty() || vValidationSet.empty())
		return 0.0;

    if (m_pTree == nullptr)
        m_pTree.reset(new pcl::search::KdTree<pcl::PointXYZ>);

    if (CPointCloudRetouchManager::getInstance()->getRetouchScene().getNumPoint() != m_NumCurrentScenePoints)
    {
        auto Scene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
        m_NumCurrentScenePoints = Scene.getNumPoint();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pCloud->resize(m_NumCurrentScenePoints);

        for (int i = 0; i < m_NumCurrentScenePoints; i++)
        {
            auto Pos = Scene.getPositionAt(i);
            pcl::PointXYZ TempPoint{ Pos.x(), Pos.y(), Pos.z() };
            pCloud->points[i] = TempPoint;
        }
        m_pTree->setInputCloud(pCloud);
    }

    std::vector<Eigen::Vector3i> PointCloudColors;
    for (auto Index : vDeterminantPointSet)
        PointCloudColors.push_back(CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(Index));

    m_MainBaseColors = __adjustColorClustering(PointCloudColors, m_MaxNumMainColors);

    m_NearestPoints.resize(m_MainBaseColors.size());

    const std::size_t NumThread = std::thread::hardware_concurrency();

    for (int i = 0; i < m_NearestPoints.size(); i++)
    {
        float MinColorDifference = FLT_MAX;
        int NearestIndex = -1;

        std::vector<std::pair<float, std::size_t>> ThreadResults(NumThread, { FLT_MAX, -1 });

#pragma omp parallel for num_threads(NumThread)
        for (int k = 0; k < PointCloudColors.size(); k++)
        {
            auto ThreadId = omp_get_thread_num();

            float TempColorDifference = __calcColorDifferences(m_MainBaseColors[i], PointCloudColors[k]);
            if (TempColorDifference < ThreadResults[ThreadId].first)
            {
                ThreadResults[ThreadId].first = TempColorDifference;
                ThreadResults[ThreadId].second = vDeterminantPointSet[k];
            }
        }

        for (int k = 0; k < ThreadResults.size(); k++)
        {
            float TempColorDifference = ThreadResults[k].first;
            if (TempColorDifference < MinColorDifference)
            {
                MinColorDifference = TempColorDifference;
                NearestIndex = ThreadResults[k].second;
            }
        }

        m_NearestPoints[i] = NearestIndex;
    }

	double Score = 0.0;
	for (auto ValidationIndex : vValidationSet)
	{
		Score += evaluateFeatureMatchFactorV(ValidationIndex);
	}

	return Score / vValidationSet.size();
}

//*****************************************************************
//FUNCTION: 
double CColorFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
    //float MinColorDifference = FLT_MAX;
    //auto Color = CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(vInputPoint);
    //for (const auto& MainColor : m_MainBaseColors)
    //{
    //    float TempColorDifference = __calcColorDifferences(Color, MainColor);
    //    if (TempColorDifference < MinColorDifference)
    //        MinColorDifference = TempColorDifference;
    //}

    //if (MinColorDifference < m_ColorThreshold)
    //    return 1.0;
    //else if (MinColorDifference < 2.0 * m_ColorThreshold)
    //{
    //    return (2.0 * m_ColorThreshold - MinColorDifference) / m_ColorThreshold;
    //}
    //else
    //    return 0.0f;

    pcl::Indices Neighbors;
    std::vector<float> Distances;

    //K search
    const int NumK = 20;
    m_pTree->nearestKSearch(vInputPoint, NumK, Neighbors, Distances);
    Neighbors.push_back(vInputPoint);
    int NumPassed = 0;
    for (auto Index : Neighbors)
    {
        float MinColorDifference = FLT_MAX;
        auto Color = CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(Index);
        for (const auto& MainColor : m_MainBaseColors)
        {
            float TempColorDifference = __calcColorDifferences(Color, MainColor);
            if (TempColorDifference < MinColorDifference)
                MinColorDifference = TempColorDifference;
        }
        if (MinColorDifference < m_ColorThreshold)
            NumPassed++;
    }
    return (double)NumPassed / Neighbors.size();
}

//*****************************************************************
//FUNCTION: 
std::string CColorFeature::outputDebugInfosV(pcl::index_t vIndex) const
{
    std::string Infos;
    Infos += _FORMAT_STR1("\nColor Feature:\nNum Main Colors are %1%, They are\n", m_MainBaseColors.size());
    for (auto& Color : m_MainBaseColors)
        Infos += _FORMAT_STR3("%1%, %2%, %3%\n", Color.x(), Color.y(), Color.z());
    auto Color = CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(vIndex);
    Infos += _FORMAT_STR3("Point's Color is: %1%, %2%, %3%\n", Color.x(), Color.y(), Color.z());
    Infos += _FORMAT_STR1("Similarity is: %1%\n", const_cast<CColorFeature*>(this)->evaluateFeatureMatchFactorV(vIndex));

    return Infos;
}

//*****************************************************************
//FUNCTION: 
std::vector<Eigen::Vector3i> CColorFeature::__adjustColorClustering(const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vMaxNumCluster) const
{
    _ASSERTE(!vColorSet.empty());
    _ASSERTE(vMaxNumCluster != 0 && vMaxNumCluster != -1);

    std::vector<std::vector<Eigen::Vector3i>> ClusterResults;
    std::vector<float> AdjustCoefficients;

    const int IterCount = 30;
    for (int CurrentK = 1; CurrentK <= vMaxNumCluster; CurrentK++)
    {
        std::vector<SColorCluster> Clusters;
        __kMeansClustering(Clusters, vColorSet, CurrentK, IterCount);

        __fillClusterCoefficient(Clusters, vColorSet);

        float SumCoefficient = 0.0f;
        std::size_t NumClusterPoints = 0;
        for (auto& Cluster : Clusters)
        {
            SumCoefficient += Cluster.Coefficient;
            NumClusterPoints += Cluster.Indices.size();
        }

        //用聚成类的点数加权
        AdjustCoefficients.push_back((SumCoefficient / CurrentK) * ((float)NumClusterPoints / vColorSet.size()));

        std::vector<Eigen::Vector3i> Centroids;
        for (auto& Cluster : Clusters)
            Centroids.push_back(Cluster.Centroid);
        ClusterResults.push_back(Centroids);
    }

    //返回自适应决定的K种主颜色
    int MaxIndex = 0;
    for (int i = 1; i < vMaxNumCluster; i++)
        if (AdjustCoefficients[i] > AdjustCoefficients[MaxIndex])
            MaxIndex = i;
    return ClusterResults[MaxIndex];
}

//*****************************************************************
//FUNCTION: 
void CColorFeature::__kMeansClustering(std::vector<SColorCluster>& voClusters, const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vK, std::size_t vIterCount) const
{
    //初始化
    voClusters.resize(vK);
    std::vector<std::pair<Eigen::Vector3i*, Eigen::Vector3i>> TagAndColorSet(vColorSet.size(), { nullptr, Eigen::Vector3i() });
    for (size_t i = 0; i < TagAndColorSet.size(); i++)
    {
        auto& [Tag, Color] = TagAndColorSet[i];
        Color = vColorSet[i];
    }

    //生成初始主颜色
    std::vector<Eigen::Vector3i> ClusterCentroids;
    for (int i = 0; i < vK; i++)
    {
        Eigen::Vector3i SeedColor;
        int Num = 0;
        float MinColorDifference = FLT_MAX;
        const int MaxAttemptNum = 30;

        do
        {
            MinColorDifference = FLT_MAX;
            SeedColor = TagAndColorSet[hiveMath::hiveGenerateRandomInteger(std::size_t(0), TagAndColorSet.size() - 1)].second;
            for (int k = 0; k < i; k++)
            {
                float TempDifference = __calcColorDifferences(SeedColor, ClusterCentroids[k]);
                if (TempDifference < MinColorDifference)
                    MinColorDifference = TempDifference;
            }
            Num++;

        } while (MinColorDifference < 2 * m_ColorThreshold && Num < MaxAttemptNum);

        ClusterCentroids.push_back(SeedColor);
    }

    //聚类迭代
    for (std::size_t i = 0; i < vIterCount; i++)
    {
        for (auto& [Tag, Color] : TagAndColorSet)
        {
            std::pair<float, Eigen::Vector3i*> MinPair(FLT_MAX, nullptr);
            for (auto& Centroid : ClusterCentroids)
            {
                float ColorDifference = __calcColorDifferences(Color, Centroid);

                if (ColorDifference < 0.5 * m_ColorThreshold)
                    MinPair = std::min(MinPair, std::make_pair(ColorDifference, &Centroid));
            }
            Tag = MinPair.second;
        }

        auto calCentroid = [](const std::vector<Eigen::Vector3i>& vClusterColorSet) -> Eigen::Vector3i
        {
            Eigen::Vector3i Centroid(0, 0, 0);
            if (vClusterColorSet.empty())
                return Centroid;

            for (auto& Color : vClusterColorSet)
                Centroid += Color;
            Centroid /= vClusterColorSet.size();
            return Centroid;
        };

        for (auto& Centroid : ClusterCentroids)
        {
            std::vector<Eigen::Vector3i> ClusterPointsData;
            for (auto& [Tag, Color] : TagAndColorSet)
                if (Tag == &Centroid)
                    ClusterPointsData.push_back(Color);
            Centroid = calCentroid(ClusterPointsData);
        }
    }

    //统计各cluster主颜色及索引
    for (int i = 0; i < ClusterCentroids.size(); i++)
        voClusters[i].Centroid = ClusterCentroids[i];

    auto Ptr = ClusterCentroids.data();
    for (int i = 0; i < TagAndColorSet.size(); i++)
        if (TagAndColorSet[i].first)
            voClusters[int(TagAndColorSet[i].first - Ptr)].Indices.push_back(i);
}

//*****************************************************************
//FUNCTION: 
void CColorFeature::__fillClusterCoefficient(std::vector<SColorCluster>& vioClusters, const std::vector<Eigen::Vector3i>& vColorSet) const
{
    //计算Silhouette Coefficient，1时为特例
    if (vioClusters.size() == 1)
    {
        if (vioClusters[0].Indices.size() >= 0.9f * vColorSet.size()) //如果聚成的簇里有绝大多数点
            vioClusters[0].Coefficient = 1.0f;
        else
            vioClusters[0].Coefficient = 0.0f;
    }
    else
    {
        for (int i = 0; i < vioClusters.size(); i++)
        {
            auto& ClusterIndices = vioClusters[i].Indices;

            if (!ClusterIndices.empty())
            {
                auto calSilhouetteCoefficient = [&]() -> float
                {
                    //轮廓系数算cluster内部色差
                    float Difference = 0.0f;
                    for (auto Index : ClusterIndices)
                        Difference += __calcColorDifferences(vioClusters[i].Centroid, vColorSet[Index]);
                    Difference /= ClusterIndices.size();

                    //轮廓系数找最近的cluster
                    float MinDifference = FLT_MAX;
                    int MinCluster = -1;
                    for (int k = 0; k < vioClusters.size(); k++)
                    {
                        if (k != i)
                        {
                            auto Temp = __calcColorDifferences(vioClusters[i].Centroid, vioClusters[k].Centroid);
                            if (Temp < MinDifference && !vioClusters[k].Indices.empty())
                            {
                                MinDifference = Temp;
                                MinCluster = k;
                            }
                        }
                    }

                    //轮廓系数算最近的cluster的色差
                    auto& NearestCluster = vioClusters[MinCluster].Indices;
                    float NearestDifference = 0.0f;
                    for (auto Index : NearestCluster)
                    {
                        NearestDifference += __calcColorDifferences(vioClusters[i].Centroid, vColorSet[Index]);
                    }
                    NearestDifference /= NearestCluster.size();

                    return (NearestDifference - Difference) / std::max(NearestDifference, Difference);
                };

                vioClusters[i].Coefficient = calSilhouetteCoefficient();
            }
        }
    }
}

//*****************************************************************
//FUNCTION: 
float CColorFeature::__calcColorDifferences(const Eigen::Vector3i& vLColor, const Eigen::Vector3i& vRColor) const
{
    Eigen::Vector3f LRGBColor{ static_cast<float>(vLColor[0]),static_cast<float>(vLColor[1]),static_cast<float>(vLColor[2]) };
    Eigen::Vector3f RRGBColor{ static_cast<float>(vRColor[0]),static_cast<float>(vRColor[1]),static_cast<float>(vRColor[2]) };
    return __calculateCIEDE2000(__RGB2LAB(LRGBColor), __RGB2LAB(RRGBColor));
}

//*****************************************************************
//FUNCTION: 
float CColorFeature::__calculateCIEDE2000(const LAB& lab1, const LAB& lab2) const
{
    const float k_L = 1.0, k_C = 1.0, k_H = 1.0;
    const float deg360InRad = deg2Rad(360.0);
    const float deg180InRad = deg2Rad(180.0);
    const float pow25To7 = 6103515625.0;

    float C1 = sqrt((lab1.a * lab1.a) + (lab1.b * lab1.b));
    float C2 = sqrt((lab2.a * lab2.a) + (lab2.b * lab2.b));

    float barC = (C1 + C2) / 2.0;

    float G = 0.5 * (1 - sqrt(pow(barC, 7) / (pow(barC, 7) + pow25To7)));

    float a1Prime = (1.0 + G) * lab1.a;
    float a2Prime = (1.0 + G) * lab2.a;

    float CPrime1 = sqrt((a1Prime * a1Prime) + (lab1.b * lab1.b));
    float CPrime2 = sqrt((a2Prime * a2Prime) + (lab2.b * lab2.b));

    float hPrime1;
    if (lab1.b == 0 && a1Prime == 0)
        hPrime1 = 0.0;
    else {
        hPrime1 = atan2(lab1.b, a1Prime);

        if (hPrime1 < 0)
            hPrime1 += deg360InRad;
    }
    float hPrime2;
    if (lab2.b == 0 && a2Prime == 0)
        hPrime2 = 0.0;
    else {
        hPrime2 = atan2(lab2.b, a2Prime);

        if (hPrime2 < 0)
            hPrime2 += deg360InRad;
    }

    float deltaLPrime = lab2.l - lab1.l;

    float deltaCPrime = CPrime2 - CPrime1;

    float deltahPrime;
    float CPrimeProduct = CPrime1 * CPrime2;
    if (CPrimeProduct == 0)
        deltahPrime = 0;
    else {

        deltahPrime = hPrime2 - hPrime1;
        if (deltahPrime < -deg180InRad)
            deltahPrime += deg360InRad;
        else if (deltahPrime > deg180InRad)
            deltahPrime -= deg360InRad;
    }

    float deltaHPrime = 2.0 * sqrt(CPrimeProduct) *
        sin(deltahPrime / 2.0);

    float barLPrime = (lab1.l + lab2.l) / 2.0;

    float barCPrime = (CPrime1 + CPrime2) / 2.0;

    float barhPrime, hPrimeSum = hPrime1 + hPrime2;
    if (CPrime1 * CPrime2 == 0) {
        barhPrime = hPrimeSum;
    }
    else {
        if (fabs(hPrime1 - hPrime2) <= deg180InRad)
            barhPrime = hPrimeSum / 2.0;
        else {
            if (hPrimeSum < deg360InRad)
                barhPrime = (hPrimeSum + deg360InRad) / 2.0;
            else
                barhPrime = (hPrimeSum - deg360InRad) / 2.0;
        }
    }

    float T = 1.0 - (0.17 * cos(barhPrime - deg2Rad(30.0))) +
        (0.24 * cos(2.0 * barhPrime)) +
        (0.32 * cos((3.0 * barhPrime) + deg2Rad(6.0))) -
        (0.20 * cos((4.0 * barhPrime) - deg2Rad(63.0)));

    float deltaTheta = deg2Rad(30.0) *
        exp(-pow((barhPrime - deg2Rad(275.0)) / deg2Rad(25.0), 2.0));

    float R_C = 2.0 * sqrt(pow(barCPrime, 7.0) /
        (pow(barCPrime, 7.0) + pow25To7));

    float S_L = 1 + ((0.015 * pow(barLPrime - 50.0, 2.0)) /
        sqrt(20 + pow(barLPrime - 50.0, 2.0)));

    float S_C = 1 + (0.045 * barCPrime);

    float S_H = 1 + (0.015 * barCPrime * T);

    float R_T = (-sin(2.0 * deltaTheta)) * R_C;

    float deltaE = sqrt(
        pow(deltaLPrime / (k_L * S_L), 2.0) +
        pow(deltaCPrime / (k_C * S_C), 2.0) +
        pow(deltaHPrime / (k_H * S_H), 2.0) +
        (R_T * (deltaCPrime / (k_C * S_C)) * (deltaHPrime / (k_H * S_H))));

    return (deltaE);
}

//*****************************************************************
//FUNCTION: 
LAB CColorFeature::__RGB2LAB(const Eigen::Vector3f& vRGBColor) const
{
    float B = gamma(vRGBColor[2] / 255.0f);
    float G = gamma(vRGBColor[1] / 255.0f);
    float R = gamma(vRGBColor[0] / 255.0f);

    float X = 0.412453 * R + 0.357580 * G + 0.180423 * B;
    float Y = 0.212671 * R + 0.715160 * G + 0.072169 * B;
    float Z = 0.019334 * R + 0.119193 * G + 0.950227 * B;

    X /= 0.95047;
    Y /= 1.0;
    Z /= 1.08883;

    float FX = X > 0.008856f ? pow(X, 1.0f / 3.0f) : (7.787f * X + 0.137931f);
    float FY = Y > 0.008856f ? pow(Y, 1.0f / 3.0f) : (7.787f * Y + 0.137931f);
    float FZ = Z > 0.008856f ? pow(Z, 1.0f / 3.0f) : (7.787f * Z + 0.137931f);

    LAB Lab;
    Lab.l = Y > 0.008856f ? (116.0f * FY - 16.0f) : (903.3f * Y);
    Lab.a = 500.f * (FX - FY);
    Lab.b = 200.f * (FY - FZ);

    return Lab;
}
