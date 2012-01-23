#include "../include/DepthImageAnalyzer/depthimageanalyzer.hpp"
namespace DepthImageAnalyzer {
  DepthImageAnalyzer::DepthImageAnalyzer(int argc, char** argv, QWidget *parent)
      : QWidget(parent)
      , qnode(argc,argv)
  {
      ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
      QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  }

  DepthImageAnalyzer::~DepthImageAnalyzer()
  {

  }

  void DepthImageAnalyzer::on_pushButton_clicked()
  {

  }

  void DepthImageAnalyzer::on_lineEdit_editingFinished()
  {

  }

  void DepthImageAnalyzer::on_lineEdit_2_editingFinished()
  {

  }
}