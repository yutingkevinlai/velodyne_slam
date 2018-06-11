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
  _pubPrevCloud = node.advertise<sensor_msgs::PointCloud2> ("/prev_cloud", 2);
  _pubCurCloud = node.advertise<sensor_msgs::PointCloud2> ("/cur_cloud", 2);

	 // subscribe to point cloud 2 and 3 topics
  _subPrevCloud = node.subscribe<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_3", 2, &MotionRemoval::prevCloudHandler, this);

  _subCurCloud = node.subscribe<sensor_msgs::PointCloud2>
						("/velodyne_cloud_2", 1000, &MotionRemoval::curCloudHandler, this);

  return true;
}

void MotionRemoval::prevCloudHandler(const sensor_msgs::PointCloud2ConstPtr& prevCloudMsg)
{
  _timePrevCloud = prevCloudMsg->header.stamp;
  //pcl::PointCloud<pcl::PointXYZI> prevCloud;
  pcl::fromROSMsg(*prevCloudMsg, _prevCloud);
  
  segmentation(_prevCloud);
  //ROS_INFO("prevTime: %f", _timePrevCloud);
  //publishCloudMsg(_pubPrevCloud, _prevCloud, _timePrevCloud, "/camera");
}

void MotionRemoval::curCloudHandler(const sensor_msgs::PointCloud2ConstPtr& curCloudMsg)
{
  _timeCurCloud = curCloudMsg->header.stamp;
  //pcl::PointCloud<pcl::PointXYZI> curCloud;
  pcl::fromROSMsg(*curCloudMsg, _curCloud);
  
  publishCloudMsg(_pubCurCloud, _curCloud, _timeCurCloud, "/camera");
}

void MotionRemoval::segmentation(const pcl::PointCloud<pcl::PointXYZ>& cloudIn)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = cloudIn;

  std::cout << "Before : " << cloud->points.size () << " points." << std::endl;
  
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  //std::cout << "After filtering : " << cloud_filtered->points.size() << " points." << std::endl;

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
  //pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
  std::cout << "After filtering : " << cloud_filtered->points.size() << " points." << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (5000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);


  std::cout << "Cluster number: " << cluster_indices.size() << std::endl;
  
  /*
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    j++;
  }
  */


}


void MotionRemoval::process()
{

}

void MotionRemoval::publishResult()
{
  ros::Time sweepTime = _timeCurCloud;
  
  
}

} // end namespace loam
