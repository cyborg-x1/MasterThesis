#ifndef KINECTTOOLS_H_
#define KINECTTOOLS_H_

#include <set>
#include <iomanip>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <iterator>
#include <math.h>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>

namespace KinTo
{
	typedef cv::Vec<short, 1> Vec1shrt;
	typedef cv::Vec<short, 2> Vec2shrt;
	typedef cv::Vec<short, 3> Vec3shrt;
	typedef cv::Vec<uchar, 3> Vec3uchar;
	typedef cv::Vec<int, 2> Vec2int;
	typedef cv::Vec<int, 3> Vec3int;
	/**
	 * This function converts a kinect depth image into steps
	 *  @param [in] src The source image (CV_16UC1)
	 *  @param [out] dst The destination image(CV_16UC1)
	 */
	void convertKinectRawToSteps(const cv::Mat &src, cv::Mat &dst);

	/**
	 * This function converts a step image into kinect depth image
	 *  @param [in] src The source image (CV_16UC1)
	 *  @param [out] dst The destination image(CV_16UC1)
	 */
	void convertStepsToKinectRaw(const cv::Mat &src, cv::Mat &dst);

	/**
	 * This function searches for the interesting part of an image.
	 * It creates a cv::Rect which includes all non-zero pixels.
	 * @param [in] src The Mat to create the ROI for
	 * @return ROI
	 */
	cv::Rect roiFinder(const cv::Mat &src);

	/**
	 * This function reduces a depth image to a given range
	 * @param[in] src Source image
	 * @param[out]dst Destination image
	 * @param[in] min_range The minimal distance allowed
	 * @param[in] max_range The maximal distance allowed
	 */
	void RangeFilter(const cv::Mat &src, cv::Mat &dst, short min_range, short max_range);

	/**
	 * This function fills holes in the depth map, but only if the difference
	 * between the pixel to the pixel on the end of the gap is
	 * smaller or equal to the amount o pixels between them.
	 */
	void gapStepMapGapFiller(const cv::Mat &src, cv::Mat &dst, uchar max_size);

	/**
	 * This function creates a neighborhood map of the step map
	 * In the first value of each pixel there will be a 8 bit value
	 * indicating which pixel is a close neighbor and belongs to the
	 * same object as the current pixel.
	 *
	 * The are for the following pixels: <br/>
	 * <table>
	 * 	<tr>
	 * 		<td>0</td><td>1</td><td>2</td>
	 * 	</tr>
	 * <tr>
	 * 		<td>7</td><td>X</td><td>3</td>
	 * 	</tr>
	 * 	<tr>
	 * 		<td>6</td><td>5</td><td>4</td>
	 * 	</tr>
	 * </table>
	 *
	 * <br/>
	 *
	 * The second value is the amount of neighbors belonging to the same object as the current pixel.
	 * The function also creates a map of the real xy coordinates of the pixels in mm
	 *
	 * @param src The source image
	 * @param map_out The neighborhood map
	 * @param threshold The biggest difference a pixel from the current can have to be a close neighbor.
	 * @param xy_coords outputs a map containing x and y coordinates of the pixels (mm)
	 *
	 */
	void createRelationNeighbourhoodMap(const cv::Mat &src, cv::Mat &map_out, unsigned short threshold=4);


	/**
	 * Blur Filter for a step map for a already available neighbormap
	 * @param[in] src input depth image (must be a step map!)
	 * @param[out] dst output depth image (step map)
	 * @param[in] neighbors The neigbormap
	 */
	void stepMapBlur(const cv::Mat &src, cv::Mat &neigbors ,cv::Mat &dst);


	/**
	 * This creates a mat with x and y values of the pixels in the depth image
	 * @param[in] src The kinect raw picture (BLURED!!)
	 * @param[in] info_msg The camera info message
	 * @param[out] xy The output (cv::Mat int[2] -> px*100)
	 */
	void createXYMap(const cv::Mat &src, const sensor_msgs::CameraInfoConstPtr& info_msg, cv::Mat &xy);

	/**
	 * This computes the normals out of the depth images, but requires a neighborhood map.
	 * @param[in] src The kinect raw picture (BLURED!!)
	 * @param[in] neighbor_map neighbor map of the raw picture
	 * @param[in] xy The xy values of the blured raw image
	 * @param[out] normals the normal vector values
	 */
	void createNormalMap(const cv::Mat &src, const cv::Mat &neighbor_map, const cv::Mat &xy, cv::Mat &normals);

	/**
	 *
	 */
	void blurDepth(const cv::Mat &src, cv::Mat &dst);

	/**
	 *
	 */
	void rgbNormals(const cv::Mat &src, cv::Mat &dst, int thres_min, int thres_max);

	void createAngleMap(const cv::Mat &normals, cv::Mat &angles);

	void crossDepthBlur(const cv::Mat &depth, const cv::Mat &neighbors, cv::Mat &depth_out, int max_size);

	void crossNormalBlur(const cv::Mat &normals, const cv::Mat &neighbors, cv::Mat &normals_out, int max_size);

	void ironFilter(const cv::Mat &depth, const cv::Mat &steps, cv::Mat &depth_out);

	void crossAnglesBlur(const cv::Mat &angles, const cv::Mat &neighbors, cv::Mat &angles_out, int max_size);

	void anglesFilter(const cv::Mat &angles, cv::Mat &angles_out, unsigned int x_angle_min,unsigned int x_angle_max,unsigned int y_angle_min,unsigned int y_angle_max,unsigned int z_angle_min,unsigned int z_angle_max, bool binary=true);

	void XYZrangeFilter(const cv::Mat &depth, const cv::Mat &xy, cv::Mat &depth_out, int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);

} /* namespace KinTo */
#endif /* KINECTTOOLS_H_ */
