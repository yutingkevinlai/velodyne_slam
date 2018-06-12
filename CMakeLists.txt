cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

set(CMAKE_CXX_STANDARD 11)

set(segmapper_DIR ~/catkin_segmap/devel/share/segmapper/cmake)
set(doxygen_catkin_DIR ~/catkin_segmap/devel/share/doxygen_catkin/cmake)
set(eigen_catkin_DIR ~/catkin_segmap/devel/share/eigen_catkin/cmake)
set(glog_catkin_DIR ~/catkin_segmap/devel/share/glog_catkin/cmake)
set(gflags_catkin_DIR ~/catkin_segmap/devel/share/gflags_catkin/cmake)
set(gtsam_catkin_DIR ~/catkin_segmap/devel/share/gtsam_catkin/cmake)
set(GTSAM_DIR ~/catkin_segmap/devel/lib/cmake/GTSAM)
set(laser_slam_DIR ~/catkin_segmap/devel/share/laser_slam/cmake)
set(mincurves_DIR ~/catkin_segmap/devel/share/mincurves/cmake)
set(minkindr_gtsam_DIR ~/catkin_segmap/devel/share/minkindr_gtsam/cmake)
set(minkindr_DIR ~/catkin_segmap/devel/share/minkindr/cmake)
set(eigen_checks_DIR ~/catkin_segmap/devel/share/eigen_checks/cmake)
set(laser_slam_ros_DIR ~/catkin_segmap/devel/share/laser_slam_ros/cmake)
set(libpointmatcher_ros_DIR ~/catkin_segmap/devel/share/libpointmatcher_ros/cmake)
set(pcl_DIR ~/catkin_segmap/devel/share/pcl-1.7/cmake)
set(pcl_catkin_DIR ~/catkin_segmap/devel/share/pcl_catkin/cmake)
set(segmatch_DIR ~/catkin_segmap/devel/share/segmatch/cmake)
set(segmatch_ros_DIR ~/catkin_segmap/devel/share/segmatch_ros/cmake)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  segmapper
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	#~/catkin_segmap/src/segmap/segmatch/include
	#~/catkin_segmap/src/segmap/segmatch_ros/include
	#~/catkin_segmap/src/laser_slam/laser_slam/include
	~/catkin_segmap/devel/include
	#~/catkin_segmap/src/mincurves/mincurves/include
	#~/catkin_segmap/src/minkindr_gtsam/minkindr_gtsam/include
	#~/catkin_segmap/src/minkindr/minkindr/include
	)


link_directories(~/catkin_segmap/devel/lib)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs segmapper
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES loam 
)

add_definitions( -march=native )

add_subdirectory(src/lib)

add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserOdometry src/laser_odometry_node.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserMapping src/laser_mapping_node.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(transformMaintenance src/transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(motionRemoval src/motion_removal_node.cpp src/lib/MotionRemoval.cpp)
target_link_libraries(motionRemoval ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # TODO: Download test data
  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
    MD5 3d5194e6981975588b7a93caebf79ba4)
  add_custom_target(${PROJECT_NAME}_test_data
    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
  configure_file(tests/loam.test.in
                 ${PROJECT_BINARY_DIR}/test/loam.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
    DEPENDENCIES
      ${PROJECT_NAME}_test_data
      multiScanRegistration
      laserOdometry
      laserMapping
      transformMaintenance
						motionRemoval)
endif()


