cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)




#Add sources
rosbuild_add_executable(surfaceDetect src/surfaceDetect.cpp)
rosbuild_add_executable(fromCompress src/fromCompress.cpp)
rosbuild_add_executable(toCompress src/toCompress.cpp)
rosbuild_add_executable(disturbance_filter_calibrator src/disturbance_filter_calibrator.cpp src/DiscreteFillAndSmoothFilter.cpp)
rosbuild_link_boost(disturbance_filter_calibrator thread regex signals filesystem system)
rosbuild_link_boost(toCompress thread regex signals filesystem system)
rosbuild_link_boost(fromCompress thread regex signals filesystem system)
rosbuild_link_boost(surfaceDetect thread regex signals filesystem system)

find_package(OpenCV REQUIRED)
target_link_libraries(disturbance_filter_calibrator ${OpenCV_LIBRARIES})
target_link_libraries(fromCompress ${OpenCV_LIBRARIES})
target_link_libraries(toCompress ${OpenCV_LIBRARIES})
target_link_libraries(surfaceDetect ${OpenCV_LIBRARIES})

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure REQUIRED)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
