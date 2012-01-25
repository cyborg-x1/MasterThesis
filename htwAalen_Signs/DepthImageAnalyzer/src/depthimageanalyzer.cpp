#include "../include/DepthImageAnalyzer/depthimageanalyzer.hpp"
namespace DepthImageAnalyzer
{
DepthImageAnalyzer::DepthImageAnalyzer(int argc, char** argv, QWidget *parent) :
		QWidget(parent), qnode(argc, argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(image(QImage)), this,SLOT(depthImage(QImage)));
	QObject::connect(&qnode, SIGNAL(currentPosValue(QPoint, float)), this,SLOT(clickPos(QPoint, float)));
	QObject::connect(this, SIGNAL(highlightDisable()), &qnode,SLOT(highlightDisable()));
	QObject::connect(this, SIGNAL(highlight(float, float)), &qnode,SLOT(highlight(float, float)));



	this->ui.graphicsView_depthImage->setScene(new QGraphicsScene);

	ui.label_error_lower->setVisible(0);
	ui.label_error_upper->setVisible(0);

	rect_Image = this->ui.graphicsView_depthImage->scene()->addRect(
			QRectF(0, 0, 640, 480), QPen(Qt::transparent), QBrush());

	MousePressCatcher *catcher = new MousePressCatcher(this,
			this->ui.graphicsView_depthImage->scene());
	rect_Image->installSceneEventFilter(catcher);

	QObject::connect(catcher, SIGNAL(clickPos(QPointF)), &qnode,SLOT(clickPos(QPointF)));



	qnode.init();
}

DepthImageAnalyzer::~DepthImageAnalyzer()
{

}

void DepthImageAnalyzer::depthImage(QImage img)
{
	rect_Image->setBrush(img);
}

void DepthImageAnalyzer::clickPos(QPoint pos, float value)
{
	int x = pos.x();
	int y = pos.y();
	this->ui.label_x->setText(QString::number(x));
	this->ui.label_y->setText(QString::number(y));
	this->ui.lineEdit_pointValue->setText(QString::number(value));
}

void DepthImageAnalyzer::on_pushButton_Highlight_Set_clicked()
{
	bool uOk,lOk;
	float upper=ui.lineEdit_highlight_upper->text().toFloat(&uOk);
	float lower=ui.lineEdit_highlight_lower->text().toFloat(&lOk);

	if(uOk && lOk)
	{
		ui.label_error_lower->setVisible(0);
		ui.label_error_upper->setVisible(0);
		if(upper >= lower)
		{
			emit highlight(lower,upper);
		}
		else
		{
			 QMessageBox msgBox;
			 msgBox.setText("Error! Lower limit is bigger than upper limit!");
			 msgBox.setIcon(QMessageBox::Warning);
			 msgBox.exec();
		}
	}
	else
	{
		if(!lOk)ui.label_error_lower->setVisible(1);
		if(!uOk)ui.label_error_upper->setVisible(1);
		emit highlightDisable();
	}

}

void DepthImageAnalyzer::on_pushButton_Highlight_Reset_clicked()
{
	ui.lineEdit_highlight_lower->setText("0");
	ui.lineEdit_highlight_upper->setText("20.0");
	emit highlightDisable();
}

}
