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
        _newCurCloud(false),
        count(0)
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
  
  _pubPrevCloud = node.advertise<sensor_msgs::PointCloud2> ("/prev_cloud", 2);
  _pubCurCloud = node.advertise<sensor_msgs::PointCloud2> ("/cur_cloud", 2);

  _subCurCloud = node.subscribe<sensor_msgs::PointCloud2>
						("/velodyne_cloud_registered", 2, &MotionRemoval::cloudHandler, this);

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
    //process();

    status = ros::ok();
    rate.sleep();
  }
}
  
void MotionRemoval::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  if (count == 0) {
    pcl::fromROSMsg(*cloudMsg, *_prevCloud);
    _timePrevCloud = cloudMsg->header.stamp;
    _newPrevCloud = true;
    count ++;
  }
  else {
    pcl::fromROSMsg(*cloudMsg, *_curCloud);
    _timeCurCloud = cloudMsg->header.stamp;
    _newCurCloud = true;
    
    // publish the cloud
    publishCloudMsg(_pubPrevCloud, *_prevCloud, _timePrevCloud, "/camera_init");
    publishCloudMsg(_pubCurCloud, *_curCloud, _timePrevCloud, "/camera_init");
    //std::cout << "prev: " << _timePrevCloud << ", cur: " << _timeCurCloud << std::endl;

    

  }
  *_prevCloud = *_curCloud;
  _timePrevCloud = _timeCurCloud;
}

/* do the calculation here */
void MotionRemoval::process()
{
  


  //std::cout << "process" << std::endl;
}

void MotionRemoval::publishResult()
{
  ros::Time sweepTime = _timeCurCloud;
  
  
}





} // end namespace loam
