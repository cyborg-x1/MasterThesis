#include "../include/DepthImageAnalyzer/depthimageanalyzer.hpp"
namespace DepthImageAnalyzer {
  DepthImageAnalyzer::DepthImageAnalyzer(int argc, char** argv, QWidget *parent)
      : QWidget(parent)
      , qnode(argc,argv)
  {
      ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
      QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
      QObject::connect(&qnode, SIGNAL(image(QImage)), this, SLOT(depthImage(QImage)));
	  this->ui.graphicsView_depthImage->setScene(new QGraphicsScene);

	  rect_Image=this->ui.graphicsView_depthImage->scene()->addRect(QRectF(0, 0,640, 480), QPen(Qt::transparent), QBrush());

	  MousePressCatcher *catcher=new MousePressCatcher(this,this->ui.graphicsView_depthImage->scene());
	  rect_Image->installSceneEventFilter(catcher);
	  QObject::connect(catcher, SIGNAL(clickPos(QPointF)), this, SLOT(clickPos(QPointF)));

	  qnode.init();
  }

  DepthImageAnalyzer::~DepthImageAnalyzer()
  {

  }

  void DepthImageAnalyzer::depthImage(QImage img)
  {
	    rect_Image->setBrush(img);
  }

  void DepthImageAnalyzer::clickPos(QPointF pos)
  {
	  int x=pos.toPoint().x();
	  int y=pos.toPoint().y();
	  this->ui.label_x->setText(QString::number(x));
	  this->ui.label_y->setText(QString::number(y));
  }
}
