#include "loam_velodyne/MotionRemoval.h"
#include "math_utils.h"

#include <geometry_msgs/PoseStamped.h>
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
  _subLaserOdomToInit = node.subscribe<nav_msgs::Odometry> ("/integrated_to_init", 5, &MotionRemoval::laserOdomToInitHandler, this);
  _pubLoamPath = node.advertise<nav_msgs::Path> ("/loam/path", 5);
// _subTf = node.subscribe<tf2_msgs::TFMessage> ("/tf", 10, &MotionRemoval::tfHandler, this);

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
  
  
  
void MotionRemoval::laserOdomToInitHandler(const nav_msgs::OdometryConstPtr& odomMsg)
{
    geometry_msgs::PoseStamped pose;
    pose.pose = odomMsg->pose.pose;
    pose.header = odomMsg->header;
    loamPath.poses.push_back(pose);
    loamPath.header = pose.header;
    _pubLoamPath.publish(loamPath);
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
