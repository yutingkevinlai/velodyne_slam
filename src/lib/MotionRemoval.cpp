#include "loam_velodyne/MotionRemoval.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

using std::cout;
using std::endl;

MotionRemoval::MotionRemoval()
    :   _prevCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _curCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _newPrevCloud(false),
{

}

MotionRemoval::~MotionRemoval()
{

}

bool MotionRemoval::setup(ros::NodeHandle& node,
                          ros::NodeHandle& privateNode)
{
	 // subscribe to point cloud 2 and 3 topics may not need
//  _subPrevCloud = node.subscribe<sensor_msgs::PointCloud2>
//      ("/velodyne_cloud_3", 2, &MotionRemoval::prevCloudHandler, this);

  _subCurCloud = node.subscribe<sensor_msgs::PointCloud2>
						("/velodyne_cloud_2", 2, &MotionRemoval::curCloudHandler, this);

  return true;
}

/* may not need it 
void MotionRemoval::prevCloudHandler(const sensor_msgs::PointCloud2ConstPtr& prevCloudMsg)
{
  _timePrevCloud = prevCloudMsg->header.stamp;
  cout << _timePrevCloud << endl;
        
}
*/
void MotionRemoval::curCloudHandler(const sensor_msgs::PointCloud2ConstPtr& curCloudMsg)
{
  _timeCurCloud = curCloudMsg->header.stamp;
  cout << _timeCurCloud << endl;
  _subCurCloud->clear();
  pcl::fromROSMsg(*curCloudMsg, _curCloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_curCloud, *_curCloud, indices);
  _newCurCloud = true;
}


void LaserOdometry::spin()
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


void MotionRemoval::process()
{

}

} // end namespace loam
