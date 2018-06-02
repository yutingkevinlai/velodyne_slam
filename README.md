<!--![Screenshot](/capture.bmp)-->
<!--Sample map built from [nsh_indoor_outdoor.bag](http://www.frc.ri.cmu.edu/~jizhang03/Datasets/nsh_indoor_outdoor.bag) (opened with [ccViewer](http://www.danielgm.net/cc/))-->

<!--:white_check_mark: Tested with ROS Indigo and Velodyne VLP16. [(Screencast)](https://youtu.be/o1cLXY-Es54)-->

# Using 3D Lidar to do Simultaneous Localization and Mapping (SLAM)

This repository is modified from previous work of [ROS Loam Velodyne](http://wiki.ros.org/loam_velodyne)

All sources were taken from [ROS documentation](http://docs.ros.org/indigo/api/loam_velodyne/html/files.html)

<!--Ask questions [here](https://github.com/laboshinl/loam_velodyne/issues/3). -->

## Build with catkin:

* Clone the repository:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/xlab905/velodyne_slam.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ~/catkin_ws/devel/setup.bash
```

* In the first terminal, run:
```
roslaunch loam_velodyne loam_velodyne.launch
```

Remember to check the velodyne configuration to modify the parameters in loam_velodyne.launch

* Play the velodyne rosbag data in the second terminal (Note the published name of the point cloud should be consistent with the subscription).
```
rosbag play ~/Downloads/velodyne.bag 
```
<!--
Or read from velodyne [VLP16 sample pcap](https://midas3.kitware.com/midas/folder/12979):
```
roslaunch velodyne_pointcloud VLP16_points.launch pcap:="/home/laboshinl/Downloads/velodyne.pcap"
```


[Quantifying Aerial LiDAR Accuracy of LOAM for Civil Engineering Applications.](https://ceen.et.byu.edu/sites/default/files/snrprojects/wolfe_derek.pdf) Derek Anthony Wolfe

[ROS & Loam_velodyne](https://ishiguro440.wordpress.com/2016/04/05/%E5%82%99%E5%BF%98%E9%8C%B2%E3%80%80ros-loam_velodyne/) 
-->
