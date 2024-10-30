#include "point_cloud_filtering/point_cloud_filtering.h"
#include "ros/node_handle.h"
#include <ros/ros.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "outlier_filter_server");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    OutlierFilter outlierFilter_server(nh, private_nh);
    ros::spin();
    return 0;

}
