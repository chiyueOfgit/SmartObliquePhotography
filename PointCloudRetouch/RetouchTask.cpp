#include "pch.h"
#include "RetouchTask.h"
#include "PointClassifier.h"
#include "PointClusterExpanderBase.h"
#include "PointClusterExpander.h"  //FIXME: 这个头文件有必要include吗？
#include "PointClusterExpanderMultithread.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
bool CRetouchTask::init(const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(vConfig);

	try 
	{
		auto NumSubconfig = vConfig->getNumSubconfig();  //FIXME：为什么就这么一句话都需要try/catch？
	}
	catch (...)
	{
		throw "vConfig Error";
		return false;
	}
//FIXME：你的程序逻辑允许重复调用init()吗？如果不允许，就要保证m_pConfig为空
	m_pConfig = vConfig;  
	m_pClusterConfig = nullptr;

//FIXME: 下面有好几个地方都出现了_HIVE_EARLY_RETURN，即解析XML可能失败，但失败后，m_pConfig仍然存有有效值，
//        按理说，如果解析失败，m_pConfig就该复原
	for (auto i = 0; i < vConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pConfig = vConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("CLUSTER")))
		{
			if (!m_pClusterConfig)
			{
				m_pClusterConfig = pConfig;
			}
			else
			{
				_HIVE_OUTPUT_WARNING("It is NOT allowed to define cluster twice.");  //FIXME：需要补一个测试用例，验证确实能处理连续定义两次CLUSTER的情况
			}
			continue;
		}
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("CLASSIFIER")))
		{
			std::optional<std::string> ClassifierSig = pConfig->getAttribute<std::string>("SIG");
			_ASSERTE(ClassifierSig.has_value());  //FIXME：这个地方用_ASSERTE()太弱了，如果外部配置文件就是没定义SIG，在release状态下就会出错

//FIXME: m_pPointClusterExpander这个命名不好，对于一个retouch task来说，这个m_pPointClusterExpander好像扯不上关系
			if (_IS_STR_IDENTICAL(ClassifierSig.value(), std::string("CLUSTER_EXPANDER_MULTITHREAD")))
				m_pPointClusterExpander = dynamic_cast<CPointClusterExpanderMultithread*>(hiveDesignPattern::hiveCreateProduct<IPointClassifier>(ClassifierSig.value()));
			else if (_IS_STR_IDENTICAL(ClassifierSig.value(), std::string("CLUSTER_EXPANDER")))
				m_pPointClusterExpander = dynamic_cast<CPointClusterExpander*>(hiveDesignPattern::hiveCreateProduct<IPointClassifier>(ClassifierSig.value()));
//FIMXE：需要补一个测试用例，证明能处理m_pPointClusterExpander没有创建成功的情况
			_HIVE_EARLY_RETURN(!m_pPointClusterExpander, _FORMAT_STR1("Fail to initialize retouch due to the failure of creating point classifier [%1%].", ClassifierSig.value()), false);
			continue;
		}
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Unknown subconfiguration type [%1%].", pConfig->getSubconfigType()));
	}

	_HIVE_EARLY_RETURN(!m_pPointClusterExpander, _FORMAT_STR1("Fail to initialize the retouch task [%1%] due to missing point classifier.", vConfig->getName()), false);

	return true;
}

void CRetouchTask::reset()
{
	m_pConfig = nullptr;
	m_pClusterConfig = nullptr;
}

//*****************************************************************
//FUNCTION: 
bool CRetouchTask::execute(const CPointCluster* vUserSpecifiedCluster) const
{
	_ASSERTE(vUserSpecifiedCluster && m_pPointClusterExpander);

//FIXME: 这里的设计有点怪，是否采用多线程，竟然会有两个完全不同的类
	if (_IS_STR_IDENTICAL(m_pPointClusterExpander->getProductSig(), std::string("CLUSTER_EXPANDER_MULTITHREAD")))
		m_pPointClusterExpander->execute<CPointClusterExpanderMultithread>(vUserSpecifiedCluster);
	else if (_IS_STR_IDENTICAL(m_pPointClusterExpander->getProductSig(), std::string("CLUSTER_EXPANDER")))
		m_pPointClusterExpander->execute<CPointClusterExpander>(vUserSpecifiedCluster);

	return m_pPointClusterExpander->getExpandPoints().size();  //FIXME：不要强制去把一个整型转换为bool，可读性不好
//FIMXE：返回值有点奇怪，如果聚类没有被扩展，会返回一个false？
}

//*****************************************************************
//FUNCTION: 
//FIXME: 函数名说的是MarkedPoints，实际返回的是ExpandPoints
//FIXME: execute()调用为什么不返回被扩展出来的点，而要把execute()和这个函数分开？
void CRetouchTask::dumpTaskMarkedPoints(std::vector<pcl::index_t>& voMarkedPoints) const
{
	voMarkedPoints = m_pPointClusterExpander->getExpandPoints();
}