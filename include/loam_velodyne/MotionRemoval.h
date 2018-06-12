#ifndef LOAM_MOTIONREMOVAL_H
#define LOAM_MOTIONREMOVAL_H

#include "common.h"
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/transform_broadcaster.h>


#include <segmatch/database.hpp>
#include <segmatch/local_map.hpp>
#include <segmatch/segmatch.hpp>

//#include <segmatch_ros/common.hpp>

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

  ros::Time _timePrevCloud;      ///< time for previous cloud (cloud 3)
  ros::Time _timeCurCloud;       ///< time for current cloud (cloud 2)
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr _prevCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _curCloud;

  bool _newPrevCloud;
  bool _newCurCloud;

  ros::Publisher _pubPrevCloud;  ///< 
  ros::Publisher _pubCurCloud;
  
  ros::Subscriber _subPrevCloud;    ///< previous cloud subscriber
  ros::Subscriber _subCurCloud;    ///< current cloud subscriber

};

} // end namespace loam


#endif //LOAM_MOTIONREMOVAL_H
