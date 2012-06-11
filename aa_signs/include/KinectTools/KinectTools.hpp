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
	typedef cv::Vec<uchar, 1> Vec1uchar;
	typedef cv::Vec<uchar, 3> Vec3uchar;
	typedef cv::Vec<int, 2> Vec2int;
	typedef cv::Vec<int, 3> Vec3int;
	typedef cv::Vec<int, 1> Vec1int;
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
	 * This function reduces a depth image to a given depth range
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
	 *
	 * EXPERIMENTAL (Problems ...)
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
	 * The second value is like the first, but does only show if the neighbor pixel is not zero.
	 *
	 * The third value is the amount of neighbors belonging to the same object as the current pixel.
	 * The function also creates a map of the real xy coordinates of the pixels in mm
	 *
	 * @param src The source image
	 * @param map_out The neighborhood map
	 * @param threshold The biggest difference another pixel can have to the current one to be a close neighbor.
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
	 * @param[in] src The depth picture (BLURED!!!)
	 * @param[in] neighbor_map neighbor map of the raw picture
	 * @param[in] xy The xy values of the blured raw image
	 * @param[out] normals the normal vector values
	 */
	void createNormalMap(const cv::Mat &src, const cv::Mat &neighbor_map, const cv::Mat &xy, cv::Mat &normals);

	/**
	 * This function calculates the angle of the normals
	 * @param[in] normals The normal map
	 * @param[out] angles The angle map
	 */
	void createAngleMap(const cv::Mat &normals, cv::Mat &angles);

	/**
	 * This function blures a depth image. It uses a given amount of pixels on the right, left, top and the bottom
	 * of the current pixel but stops at the first pixel which is no close neighbor to the one before.
	 * @param[in] depth The depth image of the kinect
	 * @param[in] neighbors The neigborhood map
	 * @param[out] depth_out The blured depth image
	 * @param max_size the maximum distance to the current pixel
	 */
	void crossDepthBlur(const cv::Mat &depth, const cv::Mat &neighbors, cv::Mat &depth_out, int max_size);

	/**
	 * This function blurs a normal map. It uses the given amount of pixels on the right,
	 * left, top and the bottom of the current pixel, but stops at the first pixel
	 * which is no close neighbor to the one before.
	 * @param[in] normals The normal image
	 * @param[in] neighbors The neigborhood map
	 * @param[out] normals_out The blured normal map
	 * @param max_size the maximum distance to the current pixel
	 */
	void crossNormalBlur(const cv::Mat &normals, const cv::Mat &neighbors, cv::Mat &normals_out, int max_size);

	/**
	 * This function blurs a angle map. It uses the given amount of pixels on the right,
	 * left, top and the bottom of the current pixel, but stops at the first pixel
	 * which is no close neighbor to the one before.
	 * @param[in] angles The angles image
	 * @param[in] neighbors The neigborhood map
	 * @param[out] angles_out The blured angles map
	 * @param max_size the maximum distance to the current pixel
	 */
	void crossAnglesBlur(const cv::Mat &angles, const cv::Mat &neighbors, cv::Mat &angles_out, int max_size);

	/**
	 * This function filters an angle map to the given angles. Every pixel lying outside the given values will be replaced
	 * by all angles set to zero in the resulting map.
	 * @param[in] angles The angles image
	 * @param[in] neighbors The xy map of each pixel
	 * @param[out] angles_out The blured angles map
	 * @param x_angle_min minimum x angle
	 * @param x_angle_max maximum x angle
	 * @param y_angle_min minimum x angle
	 * @param y_angle_max maximum x angle
	 * @param z_angle_min minimum x angle
	 * @param z_angle_max maximum x angle
	 */
	void anglesFilter(const cv::Mat &angles, cv::Mat &angles_out, unsigned int x_angle_min,unsigned int x_angle_max,unsigned int y_angle_min,unsigned int y_angle_max,unsigned int z_angle_min,unsigned int z_angle_max, bool binary=true);

	/**
	 * This function filters a depth image three dimensional. Every pixel not fitting into the giving x,y and z coordinates will be replaced by zero in the
	 * resulting depth image.
	 * param[in] depth The input depth image
	 * param[in] xy The xy map of the depth image
	 * param min_x minimum x distance from the center of the image
	 * param max_x maximum x distance from the center of the image
	 * param min_y minimum y distance from the center of the image
	 * param max_y maximum y distance from the center of the image
	 * param min_z minimum z distance from the center of the image
	 * param max_z maximum z distance from the center of the image
	 */
	void XYZrangeFilter(const cv::Mat &depth, const cv::Mat &xy, cv::Mat &depth_out, int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);









	void SurfaceExtractor(const cv::Mat &depth, const cv::Mat &pix_ok, const cv::Mat &neighbors, std::vector<cv::Rect> &rects, int minWidth=0, int minHeight=0, int maxWidth=INT_MAX, int maxHeight=INT_MAX);




	class BlobSurfaces
	{

		//Directions
		typedef enum
		{
			none=0,
			top_left=1,
			top=2,
			top_right=3,
			right=4,
			bottom_right=5,
			bottom=6,
			bottom_left=7,
			left=8
		} Directions;

		typedef unsigned int Direction;

		typedef enum
		{
			seekPix,
			checkMark,
			nextMark,
			boundaryScan_startDir,
			boundaryScan_moving,

		} States;


		//This contains region borders which are already searched
		cv::Mat regions;

		//Size of the picture
		int size_x, size_y;

		//The input mat
		cv::Mat in;

		//X and Y Of the seek process
		int y,x;

		int min_x,min_y;
		int max_x,max_y;

		std::vector<cv::Rect> &rois;

		int threshold;

	public:

		BlobSurfaces(const cv::Mat &angles_ok, std::vector<cv::Rect> &ROIs, int threshold);

	private:
		//The state machine
		void stateMachine();

		//MinMax
		void setMinMax(int x, int y);
		void resetMinMax(int x, int y);
		void addRectFromMinMax();

		Direction checkDirection(int x, int y, Direction current_dir);
		bool get_pixel_value_in_dir(int x,  int y, unsigned int dir);

	};
} /* namespace KinTo */
#endif /* KINECTTOOLS_H_ */
