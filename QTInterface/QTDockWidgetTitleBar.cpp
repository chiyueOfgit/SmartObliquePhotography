#include "pch.h"
#include "QTDockWidgetTitleBar.h"
#include <qbitmap.h>

QTDockWidgetTitleBar::QTDockWidgetTitleBar(QWidget *parent)
	: QWidget(parent)
{
	//m_pMinPix = style()->standardPixmap(QStyle::SP_TitleBarMinButton);
	m_pClosePix = style()->standardPixmap(QStyle::SP_TitleBarCloseButton);
	m_pFloatPix = style()->standardPixmap(QStyle::SP_TitleBarNormalButton);
}

QTDockWidgetTitleBar::~QTDockWidgetTitleBar()
{
}

QSize QTDockWidgetTitleBar::minimumSizeHint() const
{
	QDockWidget* DockWidget = qobject_cast<QDockWidget*>(parentWidget());

	Q_ASSERT(DockWidget != 0);

	QSize result(100, 30);

	if (DockWidget->features() & QDockWidget::DockWidgetVerticalTitleBar)
	{
		result.transpose();
	}

	return result;
}

void QTDockWidgetTitleBar::paintEvent(QPaintEvent* vEvent)
{
	QPainter painter(this);
	QRect rect = this->rect();
	QDockWidget* DockWidget = qobject_cast<QDockWidget*>(parentWidget());

	Q_ASSERT(DockWidget != 0);

	if (DockWidget->features() & QDockWidget::DockWidgetVerticalTitleBar)
	{
		QSize size = rect.size();
		size.transpose();
		rect.setSize(size);

		painter.translate(rect.left(), rect.top() + rect.width());
		painter.rotate(-90);
		painter.translate(-rect.left(), -rect.top());
	}

	painter.fillRect(rect.left(), rect.top(), rect.width(), rect.height(), m_pBackgroundColor);
	painter.drawPixmap(rect.topRight() - QPoint(m_pClosePix.width() + 10, -10), m_pClosePix);
	painter.drawPixmap(rect.topRight() - QPoint(m_pFloatPix.width() + 10 + m_pClosePix.width() + 10, -10), m_pFloatPix);
	painter.drawPixmap(rect.topRight() - QPoint(m_pMinPix.width() + 10 + m_pFloatPix.width() + 10 + m_pClosePix.width() + 10, -7), m_pMinPix);

	QFont font1("Microsoft YaHei", m_pFontSize, false, false);
	painter.setFont(font1);
	painter.setPen(m_pFontSize);
	painter.setPen(m_pFontColor);
	painter.drawText(rect.left() + 5, 20, m_pTitleBarText);
}

void QTDockWidgetTitleBar::mousePressEvent(QMouseEvent* vEvent)
{
	QPoint pos = vEvent->pos();
	QRect rect = this->rect();
	QDockWidget* DockWidget = qobject_cast<QDockWidget*>(parentWidget());
	Q_ASSERT(DockWidget != 0);
	if
		(DockWidget->features() & QDockWidget::DockWidgetVerticalTitleBar) {

		QPoint point = pos;
		pos.setX(rect.left() + rect.bottom() - point.y());
		pos.setY(rect.top() + point.x() - rect.left());

		QSize size = rect.size();
		size.transpose();
		rect.setSize(size);

	}
	const int buttonRight = 10;
	const int buttonWidth = 20;
	int right = rect.right() - pos.x();
	int button = (right - buttonRight) / buttonWidth;

	qDebug() << rect.right() << " --- " << pos.x() << " --- " << right << " --- " << button;

	switch
		(button) {

	case 0:
		vEvent->accept();
		DockWidget->hide();
		break;

	case 1:
		vEvent->accept();
		DockWidget->setFloating(!DockWidget->isFloating());
		break;

	case 2: {
		vEvent->accept();
		QDockWidget::DockWidgetFeatures features = DockWidget->features();
		if
			(features & QDockWidget::DockWidgetVerticalTitleBar)
			features &= ~QDockWidget::DockWidgetVerticalTitleBar;
		else
			features |= QDockWidget::DockWidgetVerticalTitleBar;

		DockWidget->setFeatures(features);
		break;

	}
	default:
		vEvent->ignore();
		break;

	}

}

void QTDockWidgetTitleBar::setAttr(QColor vBackgroundColor, QColor vFontColor, int vFontSize, QString vTitleBarText)
{
	m_pBackgroundColor = vBackgroundColor;
	m_pFontSize = vFontSize;
	m_pTitleBarText = vTitleBarText;
	m_pFontColor = vFontColor;
	update();
}

void QTDockWidgetTitleBar::updateMask()
{
}
