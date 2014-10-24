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

#include <pcl/filters/passthrough.h>


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
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>

//#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <DepthSense.hxx>


	ros::Subscriber sub;
	ros::Publisher pub;

pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 pc_data;

 
void reconstructionTriangulation(const sensor_msgs::PointCloud2 &segCloud){

  	  pcl::fromROSMsg(segCloud, *cloud);

 

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
	
	pcl::toROSMsg(*colored_cloud, pc_data);
  while (!viewer.wasStopped ())
  {
		pub.publish(pc_data);
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

 
}

	int main(int argc, char** argv){
	
	ros::init(argc, argv, "reconstruction_triangulation_node");
	ros::NodeHandle node;

	sub = node.subscribe("Planar_Segmentation",1, &reconstructionTriangulation);
  pub = node.advertise<sensor_msgs::PointCloud2>("Reconstruction_Triangulation", 1);

	ros::Rate loop_rate(4);
	while(ros::ok()){

	ros::spin();

	}

}


	




























	
