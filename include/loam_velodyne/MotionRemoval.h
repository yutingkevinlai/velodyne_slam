#ifndef LOAM_MOTIONREMOVAL_H
#define LOAM_MOTIONREMOVAL_H


#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

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

  void prevCloudHandler(const sensor_msgs::PointCloud2ConstPtr& prevCloudMsg);

		void curCloudHandler(const sensor_msgs::PointCloud2ConstPtr& curCloudMsg);

  /** \brief Handler method for mapping odometry messages.
   *
   * @param odomAftMapped the new mapping odometry
   */
  void process();

/*
protected:
  void transformAssociateToMap();
*/

private:

  ros::Time _timePrevCloud;      ///< time for previous cloud (cloud 3)
		ros::Time _timeCurCloud;       ///< time for current cloud (cloud 2)

  ros::Subscriber _subPrevCloud;    ///< (high frequency) laser odometry subscriber
  ros::Subscriber _subCurCloud;    ///< (low frequency) mapping odometry subscriber

};

} // end namespace loam


#endif //LOAM_TRANSFORMMAINTENANCE_H
