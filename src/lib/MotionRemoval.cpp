#include "loam_velodyne/MotionRemoval.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

using std::cout;
using std::endl;

MotionRemoval::MotionRemoval() : _prevCloud(),
	                                _curCloud()
{

}

MotionRemoval::~MotionRemoval()
{

}

bool MotionRemoval::setup(ros::NodeHandle& node,
                          ros::NodeHandle& privateNode)
{
  _pubTest = node.advertise<sensor_msgs::PointCloud2> ("/test", 2);

	 // subscribe to point cloud 2 and 3 topics
  _subPrevCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_3", 2, &MotionRemoval::prevCloudHandler, this);

  _subCurCloud = node.subscribe<sensor_msgs::PointCloud2>
						("/velodyne_cloud_2", 2, &MotionRemoval::curCloudHandler, this);

  return true;
}

void MotionRemoval::prevCloudHandler(const sensor_msgs::PointCloud2ConstPtr& prevCloudMsg)
{
		_timePrevCloud = prevCloudMsg->header.stamp;
		//pcl::PointCloud<pcl::PointXYZI> prevCloud;
  pcl::fromROSMsg(*prevCloudMsg, _prevCloud);
		
  //ROS_INFO("prevTime: %f", _timePrevCloud);
}

void MotionRemoval::curCloudHandler(const sensor_msgs::PointCloud2ConstPtr& curCloudMsg)
{
  _timeCurCloud = curCloudMsg->header.stamp;
  //pcl::PointCloud<pcl::PointXYZI> curCloud;
  pcl::fromROSMsg(*curCloudMsg, _curCloud);
		//ROS_INFO("prevTime: %f", _timeCurCloud);
}


void MotionRemoval::process()
{

}

void MotionRemoval::publishResult()
{
  ros::Time sweepTime = _timeCurCloud;
  publishCloudMsg(_pubTest, _curCloud, sweepTime, "/camera");
}

} // end namespace loam
