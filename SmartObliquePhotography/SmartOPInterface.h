#pragma once
#include "SmartOPExport.h"

namespace hiveObliquePhotography
{
	class CMultiLayeredPointCloud;

	//ȫ���̴����������������Ϣ����vMetaFileName��ָ�����ļ��ṩ
	SMARTOP_DECLSPEC bool hiveExecuteCompleteFlow(const std::string& vMetaFileName);

	//����vMetaFileName��ָ���ļ��ṩ��ԭʼ���ƣ��ϲ���һ����ĵ������ݣ������µĸ�ʽ���зֿ�洢���ֿ�������ļ���vBlockIndexFileNameָ��
	SMARTOP_DECLSPEC bool hiveConvertUnorganizedPointCloud2MultiLayered(const std::string& vMetaFileName, const std::string& vBlockIndexFileName);  

	//���Ʒ���
	SMARTOP_DECLSPEC bool hiveSegmentPointCloud(const std::string& vBlockIndexFileName);

	bool _hiveParseMetafile(const std::string& vMetaFileName);
}