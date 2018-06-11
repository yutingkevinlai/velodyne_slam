#include <ros/ros.h>
#include "loam_velodyne/MotionRemoval.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motionRemoval");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::MotionRemoval motionRemoval;

  if (motionRemoval.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }
  return 0;
}
