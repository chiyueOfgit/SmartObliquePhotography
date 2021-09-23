# Vertex Color To Texture
## 类设计：
虚基类*ITextureBake*纹理烘焙类

基于光线投射的纹理烘焙类*CRayCastingBaker*继承于*ITextureBake*
 
 结构体*SRay*纹素采样点发射光线信息
 ```
 struct SRay
		{
			Eigen::Vector3f Origin; //光线的起点
			Eigen::Vector3f Direction; //光线的方向
		};
```

 结构体*STexelInfo*纹素信息
 ```
 struct STexelInfo
		{
			Eigen::Vector2i TexelCoord; //纹素纹理坐标
			std::vector<SRay> RaySet; //纹素内采样点发射光线信息集合
		};
```

结构体*SCandidateInfo*候选交点信息
 ```
 struct SCandidateInfo
		{
			Eigen::Vector3f Intersection; //交点的坐标
			pcl::index_t SurfelIndex; //交点所属的点云点索引
		};
```
## 接口设计：
基类虚函数`CImage bakeTexture(PointCloud_t::Ptr vPointCloud, const Eigen::Vector2i& vResolution)`接收一个用于烘焙的点云模型与纹理贴图分辨率，返回一张纹理贴图。

*CRayCastingBaker*类中

`std::vector<STexelInfo> findTexelsPerFace(const SFace& vFace, Eigen::Vector2i vResolution)`函数用于计算每一个面片中所有的纹素信息。需要计算哪些纹素有采样点在面片内，每个纹素采样点发射光线的信息。

`std::vector<SCandidateInfo> executeIntersection(const SRay& vRay)`函数用于计算对于每一个从纹素采样点发射出的光线，与点云模型相交的所有候选交点信息。

`std::array<int, 3> calcTexelColor(const std::vector<SCandidateInfo>& vCandidates)`函数根据给定的候选交点集合，计算该纹素最后的颜色。


## 伪代码
```
function bakeTexture
    for each Face ∈ Mesh do
       TexelInfoSet ← findTexelsPerFace(Face,Resolution)
       for each TexelInfo ∈ TexelInfoSet do
	      for each SampleGeneratedRay ∈ TexelInfo.RaySet do
             CandidateInfoSet ← executeIntersection(TexelInfo)
             Color ← calcTexelColor(CandidateInfoSet)
          end for
		  GenerateColorPerTexel(ColorSet);
		  GenerateTextureColor;
        end for 
    end for
end function
```



