#ifndef POINT_CLOUD_FILTERING_H
#define POINT_CLOUD_FILTERING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "point_cloud_filtering/ProcessPointCloud.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;

class VoxelGrid {
  public:
	VoxelGrid(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
	void filter_pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

  private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::Publisher pointCloud2_pub_;
	ros::Subscriber pointCloud2_sub_;
};


class OutlierFilter {
    public:
    OutlierFilter(const ros::NodeHandle &nh,
                  const ros::NodeHandle &nh_private);

    bool filter_service_callback(point_cloud_filtering::ProcessPointCloud::Request &req,
                         point_cloud_filtering::ProcessPointCloud::Response &res);

    void filter_sub_callback(const sensor_msgs::PointCloud2ConstPtr &raw_cloud_msg);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::ServiceServer server_;
        
        ros::Subscriber sub_;
        pcl::PCLPointCloud2::Ptr filtered_pcl_cloud;
};

#endif // !POINT_CLOUD_FILTERING_H
