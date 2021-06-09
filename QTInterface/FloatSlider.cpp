#include "pch.h"
#include "FloatSlider.h"

QFloatSlider::QFloatSlider(QWidget* pParent /*= NULL*/) :
	QSlider(pParent),
	m_Multiplier(10000.0)
{
	connect(this, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));

	setSingleStep(1);

	setOrientation(Qt::Horizontal);
	setFocusPolicy(Qt::NoFocus);
}

void QFloatSlider::setValue(int Value)
{
	emit valueChanged((float)Value / m_Multiplier);
}

void QFloatSlider::setValue(float Value, bool BlockSignals)
{
	QSlider::blockSignals(BlockSignals);

	QSlider::setValue(Value * m_Multiplier);

	if (!BlockSignals)
		emit valueChanged(Value);

	QSlider::blockSignals(false);
}

void QFloatSlider::setRange(float Min, float Max)
{
	QSlider::setRange(Min * m_Multiplier, Max * m_Multiplier);

	emit rangeChanged(Min, Max);
}

void QFloatSlider::setMinimum(float Min)
{
	QSlider::setMinimum(Min * m_Multiplier);

	emit rangeChanged(minimum(), maximum());
}

float QFloatSlider::minimum() const
{
	return QSlider::minimum() / m_Multiplier;
}

void QFloatSlider::setMaximum(float Max)
{
	QSlider::setMaximum(Max * m_Multiplier);

	emit rangeChanged(minimum(), maximum());
}

float QFloatSlider::maximum() const
{
	return QSlider::maximum() / m_Multiplier;
}

float QFloatSlider::value() const
{
	int Value = QSlider::value();
	return (float)Value / m_Multiplier;
}