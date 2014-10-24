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



//#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <DepthSense.hxx>



	/*Initialize Publisher and Subscriber*/
	ros::Publisher pub_seg;
	ros::Subscriber sub_raw_data;

	/*Objects for storing manipulated cloud data  */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZRGB> ); // object for storing inlier point clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB> ()); // object for storing desired points clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_extracted(new pcl::PointCloud<pcl::PointXYZRGB> ());  //object for storing extracted clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr concaveHull(new pcl::PointCloud<pcl::PointXYZRGB>); // object for displaying concave hull pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_remained(new pcl::PointCloud<pcl::PointXYZRGB>);

	/*Segmentation and algorith driven objects*/
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //object for storing model coefficients
	pcl::SACSegmentation<pcl::PointXYZRGB> segmentation; // segmentation object 
	pcl::PointIndices inlierIndices; //point indices object

	/*PointCloud2 message to be published*/
	sensor_msgs::PointCloud2 pc_data; 

	/*Segmentation distance between each segmentation threshold and max iteration*/
	double max_thresh = 0.015; //15 cm
  double max_iteration = 10;


	/*------------------------------------------------------------------------------------------------*/
	void outlierDisplay(const sensor_msgs::PointCloud2& msg){

	//Convert from PointCloud2 msg to PointCloud class object
		  pcl::fromROSMsg(msg, *new_cloud);

		ros::NodeHandle nh;


	//Start data manipulation

	segmentation.setInputCloud(new_cloud); //input the converted pointcloud 
	segmentation.setModelType(pcl::SACMODEL_PLANE); //configure object to look for a plane
	segmentation.setMethodType(pcl::SAC_RANSAC); //use RANSAC method model
	segmentation.setDistanceThreshold(max_thresh); // how close a point must be to the model in order to be considered an inlier.
	segmentation.setMaxIterations(max_iteration);
	segmentation.setOptimizeCoefficients(true); //enable model coefficient refinement (optional)
	segmentation.setEpsAngle(30.0f *(M_PI/180.0f));
	segmentation.segment(inlierIndices, *coefficients);
		/*if(inlierIndices.indices.size() == 0)
			std::cout<<"Could not find any points that fitted the plane model."<<std::endl;
		else
		{
		//	for(size_t j=0; j <coefficients->values.size(); j++){

			 std::cerr<<"Model Coefficients: "<<static_cast<float>(coefficients->values[0])<<" "
							  <<static_cast<float>(coefficients->values[1])<<" "
							  <<static_cast<float>(coefficients->values[2])<<" "
							  <<static_cast<float>(coefficients->values[3])<<" "<<std::endl;

	}*/



	//Copy all of the inliers of the model to another cloud
	pcl::copyPointCloud<pcl::PointXYZRGB>(*new_cloud, inlierIndices, *inlierPoints);


	//Object to store the indices
	pcl::PointIndices::Ptr pointIndices (new pcl::PointIndices);
	//pcl::PointIndices::Ptr indices_removed(new pcl::PointIndices);
	//Object for extracting points form a list of indices


	pcl::PassThrough<pcl::PointXYZRGB> pass (true);
	pass.setInputCloud(inlierPoints);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0,1);
	pass.filter (*plane_extracted); 
	const pcl::IndicesConstPtr &indices = pass.getRemovedIndices();
	int size = indices->size(); 




	std::cerr<<"The points in this plane are: "<<plane_extracted->width*plane_extracted->height<<" data points."<<std::endl;


	//Convert from PointCloud class to PointCloud2 msg
	pcl::toROSMsg(*plane_extracted, pc_data);


	pub_seg.publish(pc_data);
	
}

	/*------------------------------------------------------------------------------------------------*/
	int main (int argc, char** argv){

	//Initiate ROS

	ros::init(argc, argv,"outliers_display_node");
	ros::NodeHandle node;


	//Subscribe to raw data topic

	sub_raw_data = node.subscribe("/softkinetic_camera/depth_registered/points", 1, &outlierDisplay);
	pub_seg = node.advertise<sensor_msgs::PointCloud2>("Outliers_Display", 1);

	ros::Rate loop_rate(4);
	while(node.ok()){

	ros::spin();
	loop_rate.sleep();

	}
	return 0;
	}


