# Vertex Color To Texture
## 类设计：
虚基类*ITextureBake*纹理烘焙类

基于光线投射的纹理烘焙类*CRayCastingBaker*继承于*ITextureBake*
 

 结构体*STexelInfo*纹素信息
 ```
 struct STexelInfo
		{
			Eigen::Vector2i TexelPos; //纹素纹理坐标
			Eigen::Vector3f TexelPosInWorld; //纹素在世界坐标系中坐标
			SFace OriginFace; //纹素属于哪一个面片
		};
```

结构体*SCandidateInfo*候选交点信息
 ```
 struct SCandidateInfo
		{
			Eigen::Vector3f Pos; //交点的坐标
			pcl::index_t PointIndex; //交点所属的点云点索引
		};
```
## 接口设计：
基类虚函数`CImage bakeTexture(PointCloud_t::Ptr vPointCloud)`接收一个用于烘焙的点云模型，返回一张纹理贴图。

*CRayCastingBaker*类中

`std::vector<STexelInfo> findTexelsPerFace(const SFace& vFace, Eigen::Vector2i vResolution)`函数用于计算每一个面片中所有的纹素信息。纹素的中心在面片内即纹素在面片内。

`std::vector<SCandidateInfo> executeIntersection(const STexelInfo& vInfo)`函数用于计算对于每一个纹素，从中心发射射线与点云模型相交的所有候选交点信息。

`Eigen::Vector3i calcTexelColor(const std::vector<SCandidateInfo>& vCandidates)`函数根据给定的候选交点集合，计算该纹素最后的颜色。


## 伪代码
```
function bakeTexture
    for each Face ∈ Mesh do
       TexelInfoSet ← findTexelsPerFace(Face,Resolution)
       for each TexelInfo ∈ TexelInfoSet do
          CandidateInfoSet ← executeIntersection(TexelInfo)
          color ← calcTexelColor(CandidateInfoSet)
          GenerateTexture(TexelInfo,color)
        end for 
    end for
end function
```



