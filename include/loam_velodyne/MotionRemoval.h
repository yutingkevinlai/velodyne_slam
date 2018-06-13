#ifndef LOAM_MOTIONREMOVAL_H
#define LOAM_MOTIONREMOVAL_H

#include "common.h"
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);
  
  void process();

  void publishResult();

/*
protected:
  void transformAssociateToMap();
*/

private:
  int count; /// for counting in handler

  tf::TransformListener _tfListener;

  ros::Time _timePrevCloud;      ///< time for previous cloud (cloud 3)
  ros::Time _timeCurCloud;       ///< time for current cloud (cloud 2)
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr _prevCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _curCloud;

  bool _newPrevCloud;
  bool _newCurCloud;

  ros::Publisher _pubPrevCloud;  ///< 
  ros::Publisher _pubCurCloud;
  
  ros::Subscriber _subPrevCloud;    ///< previous cloud subscriber
  ros::Subscriber _subCurCloud;    ///< current cloud subscriber

};

} // end namespace loam


#endif //LOAM_MOTIONREMOVAL_H
