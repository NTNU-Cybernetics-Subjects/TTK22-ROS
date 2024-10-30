#include "pcl/PCLPointCloud2.h"
#include "point_cloud_filtering/point_cloud_filtering.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

OutlierFilter::OutlierFilter(const ros::NodeHandle &nh,
                             const ros::NodeHandle &nh_private) {

	this->nh_ = nh;
	this->nh_private_ = nh_private;

	std::string raw_pointCloud_topic = "/ouster/points";
	this->sub_ = this->nh_private_.subscribe(
	    raw_pointCloud_topic, 1, &OutlierFilter::filter_sub_callback, this);
	ROS_INFO("Initializing passthrough outlier filtering.");

	// service - server
	std::string service_name = "outlier_filter";
	this->server_ = nh_private_.advertiseService(
	    service_name, &OutlierFilter::filter_service_callback, this);
	ROS_INFO("Initializing outlier_filter server.");
}

// Filter pointclouds as it comes, and store it in this->filtered_pcl_cloud
void OutlierFilter::filter_sub_callback(
    const sensor_msgs::PointCloud2ConstPtr &raw_cloud_msg) {

	pcl::PCLPointCloud2::Ptr raw_plc_cloud(new pcl::PCLPointCloud2());
	pcl_conversions::toPCL(*raw_cloud_msg, *raw_plc_cloud);

	pcl::PCLPointCloud2::Ptr filterd_pcl_cloud(new pcl::PCLPointCloud2());

	// Filter
	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlier_filter;
	outlier_filter.setInputCloud(raw_plc_cloud);
	outlier_filter.setMeanK(50);
	outlier_filter.setStddevMulThresh(1.0);
	outlier_filter.filter(*this->filtered_pcl_cloud);
	ROS_INFO("filter callback");
}

// When requested answer with the latested pointcloud
bool OutlierFilter::filter_service_callback(
    point_cloud_filtering::ProcessPointCloud::Request &req,
    point_cloud_filtering::ProcessPointCloud::Response &res) {

	ROS_INFO("PointCloud requested.");
	pcl_conversions::moveFromPCL(*this->filtered_pcl_cloud,
	                             res.processed_cloud);

	return true;
}
