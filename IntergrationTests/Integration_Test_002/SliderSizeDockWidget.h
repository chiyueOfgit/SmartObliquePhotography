#pragma once
#include "ui_SliderSizeDockWidget.h"
#include "SingleStepWindow.h"
#include <QObject>
#include <qdockwidget.h>
#include <qwidget.h>
#include <qstyle.h>

class CSliderSizeDockWidget : public QDockWidget
{
	Q_OBJECT

public:
	CSliderSizeDockWidget(QWidget* vParent, hiveConfig::CHiveConfig* vPointCloudRetouchConfig, hiveConfig::CHiveConfig* vSingleStepConfig)
		: QDockWidget(vParent),
		m_pVisualizationConfig(vPointCloudRetouchConfig),
		m_pSingleStepConfig(vSingleStepConfig),
		m_pUi(std::make_shared<Ui::CSliderSizeDockWidget>())
	{
		m_pUi->setupUi(this);
		__setSize();
		__setPosition(vParent);
		__initialSliderRadius();
		__initialSliderHardness();
		__initialStepRatioBox();

		QObject::connect(m_pUi->SliderRadius, SIGNAL(valueChanged(int)), this, SLOT(onActionSliderRadiusChange()));
		QObject::connect(m_pUi->SliderHardness, SIGNAL(valueChanged(int)), this, SLOT(onActionSliderHardnessChange()));
		QObject::connect(m_pUi->StepLengthBox, SIGNAL(valueChanged(double)), this, SLOT(onActionBoxStepRatioChange()));
		
	}

private:
	std::shared_ptr<Ui::CSliderSizeDockWidget> m_pUi = nullptr;
	hiveConfig::CHiveConfig* m_pVisualizationConfig = nullptr;
	hiveConfig::CHiveConfig* m_pSingleStepConfig = nullptr;

	void __setSize();
	void __setPosition(QWidget* vParent);
	void __initialSliderRadius();
	void __initialSliderHardness();
	void __initialStepRatioBox();
public slots:
	void onActionSliderRadiusChange();
	void onActionSliderHardnessChange();
	void onActionBoxStepRatioChange();
};