#include "loam_velodyne/MotionRemoval.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

using namespace pcl_ros;

MotionRemoval::MotionRemoval()
      : _curCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _newPrevCloud(false),
        _newCurCloud(false),
        count(0)
{
  _transform << 0,  0,  1,  0,
                1,  0,  0,  0,
                0,  1,  0,  0,
                0,  0,  0,  1;
}

MotionRemoval::~MotionRemoval()
{

}

bool MotionRemoval::setup(ros::NodeHandle& node,
                          ros::NodeHandle& privateNode)
{
  _pubVelodyneCloud3 = node.advertise<sensor_msgs::PointCloud2> ("/rotate/velodyne_cloud_3", 2);
  _pubVelodyneCloud2 = node.advertise<sensor_msgs::PointCloud2> ("/rotate/velodyne_cloud_2", 2);
  _pubVelodyneCloudRegistered = node.advertise<sensor_msgs::PointCloud2> ("/rotate/velodyne_cloud_registered", 2);
  _pubLaserCloudSurround = node.advertise<sensor_msgs::PointCloud2> ("/rotate/laser_cloud_surround", 2);
  _pubLaserOdomToInit = node.advertise<nav_msgs::Odometry> ("rotate/laser_odom_to_init", 5); 

  _subVelodyneCloud3 = node.subscribe<sensor_msgs::PointCloud2>	("/velodyne_cloud_3", 2, &MotionRemoval::velodyneCloud3Handler, this);
  _subVelodyneCloud2 = node.subscribe<sensor_msgs::PointCloud2>	("/velodyne_cloud_2", 2, &MotionRemoval::velodyneCloud3Handler, this);
  _subVelodyneCloudRegistered = node.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 2, &MotionRemoval::velodyneCloudRegisteredHandler, this);
  _subLaserCloudSurround = node.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 2, &MotionRemoval::laserCloudSurroundHandler, this);
  _subLaserOdomToInit = node.subscribe<nav_msgs::Odometry> ("/laser_odom_to_init", 5, &MotionRemoval::laserOdomToInitHandler, this);
  _subTf = node.subscribe<tf2_msgs::TFMessage> ("/tf", 10, &MotionRemoval::tfHandler, this);

  return true;
}


void MotionRemoval::spin()
{
  ros::Rate rate(100);
  bool status = ros::ok();

  // loop until shutdown
  while (status) {
    ros::spinOnce();

    // try processing new data
    process();

    status = ros::ok();
    rate.sleep();
  }
}
  
void MotionRemoval::laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  pcl::fromROSMsg(*cloudMsg, *_curCloud);
  _timeCurCloud = cloudMsg->header.stamp;

  pcl::transformPointCloud(*_curCloud, *_curCloud, _transform);
  publishCloudMsg(_pubLaserCloudSurround, *_curCloud, _timeCurCloud, "/camera_init");
}

void MotionRemoval::velodyneCloud3Handler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  pcl::fromROSMsg(*cloudMsg, *_curCloud);
  _timeCurCloud = cloudMsg->header.stamp;
  pcl::transformPointCloud(*_curCloud, *_curCloud, _transform);
  publishCloudMsg(_pubVelodyneCloud3, *_curCloud, _timeCurCloud, "/rotation/camera");
}

void MotionRemoval::velodyneCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  pcl::fromROSMsg(*cloudMsg, *_curCloud);
  _timeCurCloud = cloudMsg->header.stamp;
  pcl::transformPointCloud(*_curCloud, *_curCloud, _transform);
  publishCloudMsg(_pubVelodyneCloud2, *_curCloud, _timeCurCloud, "/rotation/camera");
}

void MotionRemoval::velodyneCloudRegisteredHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  pcl::fromROSMsg(*cloudMsg, *_curCloud);
  _timeCurCloud = cloudMsg->header.stamp;
  pcl::transformPointCloud(*_curCloud, *_curCloud, _transform);
  publishCloudMsg(_pubLaserCloudSurround, *_curCloud, _timeCurCloud, "/camera_init");
}

void MotionRemoval::laserOdomToInitHandler(const nav_msgs::OdometryConstPtr& odomMsg)
{
  
}
void MotionRemoval::tfHandler(const tf2_msgs::TFMessageConstPtr& tfMsg)
{
    std::string frame_id = tfMsg->transforms[0].header.frame_id;
    std::string child_frame_id = tfMsg->transforms[0].child_frame_id;
    //ROS_INFO("frame_id: %s",frame_id.c_str());
    //ROS_INFO("child_frame_id: %s",child_frame_id.c_str());
    if(frame_id=="/camera_init" && (child_frame_id=="/camera" || child_frame_id=="/laser_odom" || child_frame_id=="/aft_mapped"))
    {
        float x, y, z, w;
        x = tfMsg->transforms[0].transform.translation.x;
        y = -tfMsg->transforms[0].transform.translation.z;
        z = tfMsg->transforms[0].transform.translation.y;
        _stampTf.setOrigin(tf::Vector3(x,y,z));
        x = tfMsg->transforms[0].transform.rotation.x;
        y = tfMsg->transforms[0].transform.rotation.z;
        z = tfMsg->transforms[0].transform.rotation.y;
        w = tfMsg->transforms[0].transform.rotation.w;
        tf::Quaternion transTmp;
        transTmp.setRPY(0,M_PI/2,0);
        _stampTf.setRotation(tf::Quaternion(x,y,z,w));
        _stampTf.frame_id_ = frame_id;
        _stampTf.child_frame_id_ = "/rotation" + child_frame_id;
        _stampTf.stamp_ = tfMsg->transforms[0].header.stamp;
        _tfBroadcaster.sendTransform(_stampTf);
    }
}
/* do the calculation here */
void MotionRemoval::process()
{

}

void MotionRemoval::publishResult()
{
  ros::Time sweepTime = _timeCurCloud;
  
  
}



} // end namespace loam
