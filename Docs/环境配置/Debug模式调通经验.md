本项目原本只能用Release模式，不能使用Debug模式，调通的路上遇到了很多问题。

---
# 项目重命名，引用却未及时更改
* 一些项目的附加包含目录仍然沿用上一版本的名称（AutoRetouch项目已改名为PointcloudRetouch）。
  * 解决方式：修正错误的附加包含目录即可。

# “_DEBUG宏”定义错误
* Debug模式下的QTInterface项目未定义“_DEBUG宏”，导致错用Release版本的QT库。
  * 解决方式：在项目属性中预处理定义处定义“_DEBUG宏”即可。
* Debug模式下的PCLVisualization项目错误定义“NDEBUG宏”，导致错用Release版本的PCL库。
  * 解决方式：在项目属性中预处理定义处将定义的“NDEBUG宏”改为“_DEBUG宏”即可。

# 运行库选项错误
* Debug模式下的PCLVisualization项目与QTInterface项目的运行库选项错误使用默认的"多线程DLL"而非“多线程调试DLL”。
  * 解决方式：运行库选项改为“多线程调试DLL”即可。
* 这两个项目Debug模式的默认运行库选项为“多线程DLL”的原因未知。
![](pics/运行库选项位置.png)

# NuGet包制作不当
* PCL与VTK的nuget包未区分Debug模式与Release模式，导致Debug模式下接触到Release模式的库文件。
  * 解决方式：将Debug模式与Release模式的库文件放到nuget包中的不同路径下。

# 不同库对VTK的Debug包需求不同
* Debug模式下PCL与QT需求的库文件名称不同。如：PCL需要vtkCommonCore-8.2-gd.dll，QT则需要vtkCommonCore-8.2D.dll。两者后缀不同，内容却是相同的。
这导致单独的一方nuget包无法满足需求。
  * <font color="#dd0000">暂时的</font>解决方式：在为QT制作的nuget包中添加PCL需要的dll文件。
<br/>
* 通过多添加一份Debug模式dll文件，解决了当前问题，但导致了文件冗余。
<font color="#dd0000">想要真正解决这个问题可能需要重新编译PCL，更改其对Debug模式dll的需求。</font>
![](pics/冗余dll文件示意图.png)