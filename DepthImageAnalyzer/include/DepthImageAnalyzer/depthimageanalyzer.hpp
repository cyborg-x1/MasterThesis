#ifndef DEPTHIMAGEANALYZER_H
#define DEPTHIMAGEANALYZER_H

#include <QtGui/QWidget>
#include <QBrush>
#include <QImage>
#include <QGraphicsRectItem>
#include <QPoint>
#include <QDebug>
#include <QMessageBox>

#include "MousePressCatcher.hpp"
#include "ui_depthimageanalyzer.h"
#include "qnode.hpp"



namespace DepthImageAnalyzer
{

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
		void on_pushButton_Highlight_Set_clicked();
		void on_pushButton_Highlight_Reset_clicked();
		void depthImage(QImage img);
		void clickPos(QPoint, float value);

	signals:
		void highlightDisable();
		void highlight(float lower, float upper);
    };

}
#endif // DEPTHIMAGEANALYZER_H
