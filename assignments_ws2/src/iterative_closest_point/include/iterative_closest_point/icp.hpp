#ifndef ICP_H
#define ICP_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/subscription.hpp>
// #include <sensor_msgs/msg/point_cloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;

class ICP_node : public rclcpp::Node {
  public:
    ICP_node();

    void raw_cloud_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr raw_cloud);

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        sub_raw_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_cloud_original_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_cloud_transfomred_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_algigned_cloud_;
};

/* Generic z-y-x transformation matrix */
Eigen::Matrix4d getFullTransformationMatrix(double roll, double pitch,
                                            double yaw, double xTrans,
                                            double yTrans, double zTrans);
/* Print a 4by4 Eigen matrix */
std::string Matrix4dToStr(Eigen::Matrix4d &matrix);

/* DownSample a pclCloud using voxel filter */
PCLPointCloud::Ptr downSample(PCLPointCloud::Ptr &raw_cloud);

/* The iterative closest point algorithm */
bool applyICM(PCLPointCloud::Ptr &pclCloudSource,
              PCLPointCloud::Ptr &pclCloudTarget);

#endif // !ICP_H
#define ICP_H
