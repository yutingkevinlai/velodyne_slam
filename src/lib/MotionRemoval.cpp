#include "loam_velodyne/MotionRemoval.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

using namespace pcl_ros;

MotionRemoval::MotionRemoval()
    :   _prevCloud(new pcl::PointCloud<pcl::PointXYZ>()),
        _curCloud(new pcl::PointCloud<pcl::PointXYZ>()),
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
						("/segmatch/source_representation", 2, &MotionRemoval::cloudHandler, this);

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
  std::cout << "get segmentation results!" << std::endl;

  pcl::fromROSMsg(*cloudMsg, *_curCloud);
  _timeCurCloud = cloudMsg->header.stamp;
  _newCurCloud = true;
  
  //std::cout << "width:" << _curCloud->width << std::endl;
  //std::cout << "height: " << _curCloud->height << std::endl;
  //std::cout << "size: " << _curCloud->points.size() << std::endl;
  
  // transform coordinate of the point cloud
  //if (_tfListener.waitForTransform("camera_init", "world", _timeCurCloud, ros::Duration(0.2))) {
  //  std::cout << "connect world to camera_init frame" << std::endl;
  //  pcl_ros::transformPointCloud("camera_init", *_curCloud, *_curCloud, _tfListener);
  //}
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  pcl::transformPointCloud(*_curCloud, *_curCloud, transform);
  
  
  std::cout << "publish the point cloud" << std::endl;
  // publish the cloud
  publishCloudMsg(_pubCurCloud, *_curCloud, _timeCurCloud, "/camera_init");
  //std::cout << "prev: " << _timePrevCloud << ", cur: " << _timeCurCloud << std::endl;

    

  
  //*_prevCloud = *_curCloud;
  //_timePrevCloud = _timeCurCloud;
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
