// #include "ros/duration.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <point_cloud_filtering/ProcessPointCloud.h>
#include <string>

int main(int argc, char **argv){
    ros::init(argc, argv, "outlierFilter_client");

    ros::NodeHandle nh;
    
    std::string service_name = "outlier_filter";
    ros::ServiceClient client = nh.serviceClient<point_cloud_filtering::ProcessPointCloud>(service_name);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud_outlier", 5);

    point_cloud_filtering::ProcessPointCloud srv;
    if (client.call(srv)){
        ROS_INFO("Requesting a PointCloud2");
        pub.publish(srv.response);
    }
    else {
        ROS_ERROR("failed to call outliner_filter");
        return 1;
    }

    ros::Duration(1.0).sleep();
    ros::spin();
    
    return 0;
    
}

