#include "iterative_closest_point/icp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <functional>
#include <memory>
#include <rclcpp/utilities.hpp>
// #include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

ICP_node::ICP_node() : Node("icp_node") {

    this->sub_raw_cloud_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/os_cloud_node/points", 1,
            std::bind(&ICP_node::raw_cloud_callback, this,
                      std::placeholders::_1));

    this->pub_cloud_original =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "icp/original_cloud", 1);
    this->pub_cloud_transfomred_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "icp/transformed_cloud", 1);
    this->pub_algigned_cloud_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "icp/algined_cloud", 1);
}

void ICP_node::raw_cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr raw_msg_cloud) {

    PCLPointCloud::Ptr pclCloudOriginal(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudTransformed(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudAligned(new PCLPointCloud);
    
    // Convert raw cloud to pcl-cloud, and downsample to lower ICP run-time
    pcl::fromROSMsg(*raw_msg_cloud, *pclCloudOriginal);
    pclCloudOriginal = downSample(pclCloudOriginal);
    //
    // // Transform the cloud to have something to do the ICM on
    // Eigen::Matrix4d transformationMatrix = getFullTransformationMatrix(0, 0, M_PI/10,
    //                                                                    0, 0.1, 0);
    // ROS_INFO("Transforming the incomming point cloud with the transformation:");
    // rosPrintMatrix4dInfo(transformationMatrix);
    // pcl::transformPointCloud(*pclCloudOriginal, *pclCloudTransformed, transformationMatrix);
    //
    // *pclCloudAligned = *pclCloudTransformed;
    //
    // // Align the point cloud with ICP
    // ROS_INFO("Applying ICM transformed->original with size = %d", pclCloudOriginal->size());
    // bool ICM_status = this->applyICM(pclCloudAligned, pclCloudOriginal);
    //
    // // Convert transformed point cloud to rosmsg
    // sensor_msgs::PointCloud2 msgTransformedCloud;
    // pcl::toROSMsg(*pclCloudTransformed, msgTransformedCloud);
    // msgTransformedCloud.header.stamp = ros::Time::now();
    //
    // // Convert aligned point cloud to rosmsg
    // sensor_msgs::PointCloud2 msgAlignedCloud;
    // pcl::toROSMsg(*pclCloudAligned, msgAlignedCloud);
    // msgAlignedCloud.header.stamp = ros::Time::now();
    //
    // // Publish all clouds at the same time for comparing
    // this->pub_cloud_original_.publish(msgCloud);
    // this->pub_cloud_transform_.publish(msgTransformedCloud);
    // this->pub_cloud_aligned_.publish(msgAlignedCloud);
    //
    // RCLCPP_INFO(this->get_logger(), "got data");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP_node>());
    rclcpp::shutdown();
    return 0;
}
