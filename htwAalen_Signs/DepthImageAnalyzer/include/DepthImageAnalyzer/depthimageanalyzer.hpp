#ifndef DEPTHIMAGEANALYZER_H
#define DEPTHIMAGEANALYZER_H

#include <QtGui/QWidget>
#include <QBrush>
#include <QImage>
#include "ui_depthimageanalyzer.h"
#include "qnode.hpp"



namespace DepthImageAnalyzer {

    class DepthImageAnalyzer : public QWidget
    {
	Q_OBJECT

    public:
	DepthImageAnalyzer(int argc, char** argv, QWidget *parent = 0);
	~DepthImageAnalyzer();

    private slots:
	void on_pushButton_clicked();

	void on_lineEdit_editingFinished();

	void on_lineEdit_2_editingFinished();

    private:
	Ui::DepthImageAnalyzerClass ui;
	QGraphicsScene scene;
	QNode qnode;
	QImage image;


	private slots:
	void depthImage(QImage img);


    };

}
#endif // DEPTHIMAGEANALYZER_H
