# SmartObliquePhotography

# 功能一览

通过倾斜摄影模型编辑软件，将复杂冗余、不可编辑的OSGB模型，处理为易于使用、只保持地势地貌的obj网格模型。软件效果演示如下图：

html <div align="center">

<img src="Docs\README Pictures\软件效果演示.gif" style="zoom:100%;" />

</div>

<center style="color:#000000">软件效果演示视频。原场景为不可编辑的OSGB模型。编辑后场景为剔除杂物后，仅保留地势地貌的obj模型。</center>

倾斜摄影模型编辑软件的**功能架构**如下图所示：

<img src="Docs\README Pictures\架构.png" style="zoom:100%;" />

<center style="color:#000000">“倾斜摄影模型编辑软件”功能架构。</center>

## 杂物去除

根据选择区域的多种特征，使用**区域生长**的方式探索场景，以达到前景与背景的分离。

html <div align="center">

<img src="Docs\README Pictures\杂物去除.gif" style="zoom:150%;" />

<center style="color:#C0C0C0">杂物去除效果演示。</center>

</div>

## 空洞填补

根据**高程图**建立高斯金字塔恢复空洞处的高程，之后以下压的方式还原颜色。

<img src="Docs\README Pictures\空洞填补.gif" alt="1" style="zoom:150%;" />

<center style="color:#C0C0C0">空洞填补效果演示。</center>

## 网格重建

基于泊松方程的网格重建。

<img src="Docs\README Pictures\网格重建.png" style="zoom:100%;" />

<center style="color:#C0C0C0">左侧：点云。右侧：网格。将点云重建为网格。重建过程中颜色信息丢失。</center>

## 网格参数化与纹理烘焙

基于**图特方法**的网格参数化与基于**光线投射**的纹理烘焙。

<img src="Docs\README Pictures\纹理烘焙.png" style="zoom:100%;" />

首先使用网格参数化方法生成网格的UV坐标，随后使用纹理烘焙得到纹理图片。最后张贴纹理至obj网格。
