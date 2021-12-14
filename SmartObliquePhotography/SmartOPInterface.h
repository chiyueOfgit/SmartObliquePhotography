#pragma once
#include "SmartOPExport.h"

namespace hiveObliquePhotography
{
	class CMultiLayeredPointCloud;

	//全过程处理，所有输入输出信息都由vMetaFileName所指定的文件提供
	SMARTOP_DECLSPEC bool hiveExecuteCompleteFlow(const std::string& vMetaFileName);

	//读入vMetaFileName所指定文件提供的原始点云，合并成一个大的点云数据，并以新的格式进行分块存储，分块的索引文件由vBlockIndexFileName指定
	SMARTOP_DECLSPEC bool hiveConvertUnorganizedPointCloud2MultiLayered(const std::string& vMetaFileName, const std::string& vBlockIndexFileName);  

	//点云分类
	SMARTOP_DECLSPEC bool hiveSegmentPointCloud(const std::string& vBlockIndexFileName);

	bool _hiveParseMetafile(const std::string& vMetaFileName);
}