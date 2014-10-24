/*
* ransac_seg.cpp
*
*  Created on: Aug 6, 2014
*     Author: Junior Cunha
*
* This node subscribe from Planar_Segmentation topic allocated from plane_seg.cpp and extracts definte planes and publishes it
*/
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

	/*Initiate ROS publisher and subscriber*/
	ros::Publisher pub_ransac;
	ros::Subscriber sub_seg;

	/*Point Distance Threshold and Max Iteratoin*/
	double dist_threshold = 0.005;
	int max_iteration = 5;
	/*Parameter for filtering point cloud noise*/
	double noise_eliminate;

	/*Radius Base and point neighbors threshold*/
	double outlier_threshold = 0.10; //15 cm
	short int neighbors = 5; //10 neighbors

	/*Objects for storing pointcloud data*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	sensor_msgs::PointCloud2 finalCloud;

	/*Different Planes*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_one(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_two(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_three(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_four(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_biggest(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_smallest(new pcl::PointCloud<pcl::PointXYZRGB>);

	/* ShutDown Request */
	bool ransac_node_shutdown = false;


	/*
	-----------------------------------------------------------------------------------------*/
	void ransacSeg(const sensor_msgs::PointCloud2 &planeCloud){


	//Convert PointCloud2 msg to PointCloud class object
	pcl::fromROSMsg(planeCloud, *planes_cloud); 

	//RANSAC objects: model and algorithm
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(planes_cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZRGB>ransac(model);

	std::cerr<<"Total points: "<<planes_cloud->width*planes_cloud->height<<std::endl;

	//Set the maximum allowed distance to the model
	ransac.setDistanceThreshold(dist_threshold);
	ransac.computeModel();
	ransac.setMaxIterations(max_iteration);



	std::vector<int> inlierIndices;


	ransac.getInliers(inlierIndices); //return the best set of inliers found so far for this model


	//Copy all inliers of the model to another cloud
	pcl::copyPointCloud<pcl::PointXYZRGB>(*planes_cloud, inlierIndices, *inlierPoints);


	//Convert the PointCloud class object to a PointCloud2 msg
	pcl::toROSMsg(*inlierPoints, finalCloud);


	pub_ransac.publish(finalCloud);




	}


	int main(int argc, char** argv){

	ros::init(argc, argv, "ransac_seg_node");
	ros::NodeHandle node;

	//Subscribe to planar segmentation
	sub_seg = node.subscribe("Planar_Segmentation",1, &ransacSeg);
	//Publish RANSAC segmentations
	pub_ransac = node.advertise<sensor_msgs::PointCloud2>("RANSAC_Segmentation", 1);



	if(ransac_node_shutdown){
		ros::shutdown();
		}
	while(1){

	ros::spin();

	}



}





