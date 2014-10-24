#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <vector>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>


//ros include files
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/PointIndices.h>
#include <pcl/ros/conversions.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>

//#include <pcl/ros/point_traits.h>//for centroid
#include <pcl/PointIndices.h>//for centroid
#include "pcl/common/impl/centroid.hpp"//for centroid

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/io/io.h> 
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

//Segmentation and Indicies Extraction PCL libraries
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/extract_indices.h>



//#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <DepthSense.hxx>


#include <sstream>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
 
/**
* This tutorial demonstrates simple sending of messages over the ROS system.
*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	void Callback(const sensor_msgs::PointCloud2 &cloud_filtered)
{

 pcl::fromROSMsg(cloud_filtered, *new_cloud_filtered);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (new_cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.005); // 0.5cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (new_cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    cloud_cluster->points.push_back (new_cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    j++;
  }


  int cluster_length = cluster_indices.size();
  std::cerr<< cluster_length << " Clusters Found." << std::endl;
/*  
  pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector4f eigen_vect;
  //Object to hold centroid values
  pcl::NdCentroidFunctor<pcl::PointXYZ>::Ptr centroid(new pcl::NdCentroidFunctor<pcl::PointCloudXYZ>);
  centroid.compute3DCentroid(center,cluster_indices,eigen_vect);
  */
  


//     centroids_pub.publish(msg);

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "euclidean_cluster_node");
  ros::NodeHandle n;

   
     // ros::Publisher centroids_pub = n.advertise<geometry_msgs::PointStamped("cluster_centroids");

      ros::Subscriber sub = n.subscribe("RANSAC_Segmentation",1, &Callback);
    

 while(1){
     ros::spin();
   }
 
 
   return 0;
}
