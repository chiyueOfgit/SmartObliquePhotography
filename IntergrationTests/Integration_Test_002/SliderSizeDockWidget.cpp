#include "pch.h"
#include "SliderSizeDockWidget.h"
#include "SingleStepConfig.h"

void CSliderSizeDockWidget::__setSize()
{
	this->setFixedSize(235, 90);
}

void CSliderSizeDockWidget::__setPosition(QWidget* vParent)
{
	QPoint ParentPoint = vParent->pos();
	QPoint p1 = vParent->mapToGlobal(ParentPoint);
	this->move(p1.x() + vParent->width() - this->width() - 12, p1.y() - 12);
}

void CSliderSizeDockWidget::__initialSliderRadius()
{
	m_pUi->SliderRadius->setMinimum(10.0);
	m_pUi->SliderRadius->setMaximum(100.0);
	m_pUi->SliderRadius->setValue(static_cast<float>(m_pVisualizationConfig->getAttribute<double>("SCREEN_CIRCLE_RADIUS").value()));
}

void CSliderSizeDockWidget::__initialSliderHardness()
{
	m_pUi->SliderHardness->setMinimum(0.0);
	m_pUi->SliderHardness->setMaximum(1.0);
	m_pUi->SliderHardness->setValue(static_cast<float>(m_pVisualizationConfig->getAttribute<double>("SCREEN_CIRCLE_HARDNESS").value()));
}

void CSliderSizeDockWidget::__initialStepRatioBox()
{
	m_pUi->StepLengthBox->setMinimum(0.0);
	m_pUi->StepLengthBox->setMaximum(100.0);
	m_pUi->StepLengthBox->setSingleStep(2.0f);
	m_pUi->StepLengthBox->setValue(m_pSingleStepConfig->getAttribute<float>("STEP_RATIO").value());
}

void CSliderSizeDockWidget::onActionSliderRadiusChange()
{
	m_pVisualizationConfig->overwriteAttribute("SCREEN_CIRCLE_RADIUS", static_cast<double>(m_pUi->SliderRadius->value()));
}

void CSliderSizeDockWidget::onActionSliderHardnessChange()
{
	m_pVisualizationConfig->overwriteAttribute("SCREEN_CIRCLE_HARDNESS", static_cast<double>(m_pUi->SliderHardness->value()));
}

void CSliderSizeDockWidget::onActionBoxStepRatioChange()
{
	m_pSingleStepConfig->overwriteAttribute(STEP_RATIO, static_cast<float>(m_pUi->StepLengthBox->value()));
}
