#ifndef LOAM_MOTIONREMOVAL_H
#define LOAM_MOTIONREMOVAL_H

#include "common.h"
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

namespace loam {

/** \brief Implementation of the LOAM transformation maintenance component.
 *
 */
class MotionRemoval {
public:
  MotionRemoval();
  ~MotionRemoval();

  /** \brief Setup component.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  bool setup(ros::NodeHandle& node,
             ros::NodeHandle& privateNode);


  /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
  void spin();

  void laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);
  void velodyneCloud3Handler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);
  void velodyneCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);
  void velodyneCloudRegisteredHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);
  void laserOdomToInitHandler(const nav_msgs::OdometryConstPtr& odomMsg);
  void tfHandler(const tf2_msgs::TFMessageConstPtr& tfMsg);

  void process();

  void publishResult();

private:
  int count; /// for counting in handler

  tf::TransformListener _tfListener;

  ros::Time _timePrevCloud;      ///< time for previous cloud (cloud 3)
  ros::Time _timeCurCloud;       ///< time for current cloud (cloud 2)
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr _prevCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _curCloud;
  nav_msgs::Odometry _odom;      ///< mapping odometry message

  bool _newPrevCloud;
  bool _newCurCloud;

  Eigen::Matrix4f _transform;
  ros::Subscriber _subVelodyneCloud3;
  ros::Subscriber _subVelodyneCloud2;
  ros::Subscriber _subVelodyneCloudRegistered;
  ros::Subscriber _subLaserCloudSurround;
  ros::Subscriber _subTf;

  ros::Publisher _pubVelodyneCloud3;
  ros::Publisher _pubVelodyneCloud2;
  ros::Publisher _pubVelodyneCloudRegistered;
  ros::Publisher _pubLaserCloudSurround;
  ros::Publisher _pubLaserOdomToInit;

  nav_msgs::Path loamPath;

  ros::Subscriber _subLaserOdomToInit;
  ros::Publisher _pubLoamPath;

  tf::TransformBroadcaster _tfBroadcaster;
  tf::StampedTransform _stampTf;
};

} // end namespace loam


#endif //LOAM_MOTIONREMOVAL_H
