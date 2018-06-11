#include "loam_velodyne/MotionRemoval.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

using std::cout;
using std::endl;

MotionRemoval::MotionRemoval()
{

}

MotionRemoval::~MotionRemoval()
{

}

bool MotionRemoval::setup(ros::NodeHandle& node,
                          ros::NodeHandle& privateNode)
{
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
		cout << _timePrevCloud << endl;
}

void MotionRemoval::curCloudHandler(const sensor_msgs::PointCloud2ConstPtr& curCloudMsg)
{
  _timeCurCloud = curCloudMsg->header.stamp;
  cout << _timeCurCloud << endl;
}


void MotionRemoval::process()
{

}

} // end namespace loam
