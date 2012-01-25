#ifndef DEPTHIMAGEANALYZER_H
#define DEPTHIMAGEANALYZER_H

#include <QtGui/QWidget>
#include <QBrush>
#include <QImage>
#include <QGraphicsRectItem>
#include <QPoint>
#include "MousePressCatcher.hpp"
#include "ui_depthimageanalyzer.h"
#include "qnode.hpp"



namespace DepthImageAnalyzer {

    class DepthImageAnalyzer : public QWidget
    {
	Q_OBJECT

    public:
	DepthImageAnalyzer(int argc, char** argv, QWidget *parent = 0);
	~DepthImageAnalyzer();

    private:
	Ui::DepthImageAnalyzerClass ui;

	QNode qnode;
	QGraphicsRectItem *rect_Image;

	private slots:
	void depthImage(QImage img);
	void clickPos(QPointF);

	signals:


    };

}
#endif // DEPTHIMAGEANALYZER_H
