/**
 * @file /include/DepthImageAnalyzer/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef DepthImageAnalyzer_QNODE_HPP_
#define DepthImageAnalyzer_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QImage>
#include <QStringListModel>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QGraphicsView>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace DepthImageAnalyzer {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	//bool init(const std::string &master_url, const std::string &host_url); //TODO delete?
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	QGraphicsView*  depthViewer(){ return &depth_viewer;}
	void log( const LogLevel &level, const std::string &msg);

signals:
	void loggingUpdated();
    void rosShutdown();
    void image(QImage);


private:
	int init_argc;
	char** init_argv;

	QGraphicsView depth_viewer;


	//DEMOSTUFF TODO DELETE
	ros::Publisher chatter_publisher;
	/////////////////////////////////


	//Loglist
	QStringListModel logging_model;

	//QImage
	QImage display_image;

	//Image subscriber and callback function
	image_transport::Subscriber depth_image;
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);


	//Storage for current depth image
	cv::Mat current_depth_image;


};

}  // namespace DepthImageAnalyzer

#endif /* DepthImageAnalyzer_QNODE_HPP_ */
