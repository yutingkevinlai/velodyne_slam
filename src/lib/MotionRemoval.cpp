#include "loam_velodyne/MotionRemoval.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

using std::cout;
using std::endl;

MotionRemoval::MotionRemoval() : count(0),
                                 _prevCloud(),
	                               _curCloud()
{

}

MotionRemoval::~MotionRemoval()
{

}

bool MotionRemoval::setup(ros::NodeHandle& node,
                          ros::NodeHandle& privateNode)
{
  _pubPrevCloud = node.advertise<sensor_msgs::PointCloud2> ("/prev_cloud", 2);
  _pubCurCloud = node.advertise<sensor_msgs::PointCloud2> ("/cur_cloud", 2);

	// subscribe to point cloud 2 and 3 topics
  //_subPrevCloud = node.subscribe<sensor_msgs::PointCloud2>
  //          ("/velodyne_cloud_3", 2, &MotionRemoval::prevCloudHandler, this);

  _subCurCloud = node.subscribe<sensor_msgs::PointCloud2>
						("/velodyne_cloud_registered", 2, &MotionRemoval::cloudHandler, this);

  return true;
}

void MotionRemoval::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  if (count == 0) {
    pcl::fromROSMsg(*cloudMsg, _prevCloud);
    _timePrevCloud = cloudMsg->header.stamp;
    count ++;
  }
  else {
    pcl::fromROSMsg(*cloudMsg, _curCloud);
    _timeCurCloud = cloudMsg->header.stamp;
    publishCloudMsg(_pubPrevCloud, _prevCloud, _timePrevCloud, "/camera_init");
    publishCloudMsg(_pubCurCloud, _curCloud, _timePrevCloud, "/camera_init");
    std::cout << "prev: " << _timePrevCloud << ", cur: " << _timeCurCloud << std::endl;
    //process
  }
  _prevCloud = _curCloud;
  _timePrevCloud = _timeCurCloud;
}


void MotionRemoval::process()
{

}

void MotionRemoval::publishResult()
{
  ros::Time sweepTime = _timeCurCloud;
  
  
}

} // end namespace loam
