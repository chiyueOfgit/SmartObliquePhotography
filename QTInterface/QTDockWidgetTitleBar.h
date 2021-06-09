#pragma once
#ifndef QTDOCKWIDGETTITLEBAR_H
#define QTDOCKWIDGETTITLEBAR_H

#include <QObject>
#include <qdockwidget.h>
#include <qwidget.h>
#include <qstyle.h>
#include <qpainter.h>
#include <qevent.h>
#include <qsize.h>
#include "qdebug.h"


class QTDockWidgetTitleBar : public QWidget
{
	Q_OBJECT

public:
	QTDockWidgetTitleBar(QWidget *parent = Q_NULLPTR);
	~QTDockWidgetTitleBar();

	void setAttr(QColor vBackgroundColor, QColor vFontColor, int vFontSize, QString vTitleBarText);
	QSize sizeHint() const { return minimumSizeHint(); };
	QSize minimumSizeHint() const;

public slots:
	void updateMask();

protected:
	void paintEvent(QPaintEvent* vEvent);
	void mousePressEvent(QMouseEvent* vEvent);

private:
	QColor m_pBackgroundColor = QColor(255, 255, 255, 255);
	QColor m_pFontColor = QColor(0, 0, 0, 255);
	QString m_pTitleBarText = "";
	QPixmap m_pMinPix, m_pClosePix, m_pFloatPix;
	
	int m_pFontSize = 10;

};


#endif