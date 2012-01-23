#ifndef DEPTHIMAGEANALYZER_H
#define DEPTHIMAGEANALYZER_H

#include <QtGui/QWidget>
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
	QNode qnode;
    };

}
#endif // DEPTHIMAGEANALYZER_H
