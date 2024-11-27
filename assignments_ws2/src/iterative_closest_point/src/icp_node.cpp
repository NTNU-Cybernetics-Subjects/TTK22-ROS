#include "iterative_closest_point/icp.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/utilities.hpp>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

// #include <functional>
// #include <memory>

ICP_node::ICP_node() : Node("icp_node") {

    // Add cloud topic as a paramter to enable easy switching at runtime
    std::string default_cloud_topic = "/os_cloud_node/points";
    this->declare_parameter("cloud_topic", default_cloud_topic);
    std::string cloud_topic = this->get_parameter("cloud_topic").as_string();

    // Subscribe to the spesificed point cloud
    sub_raw_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, 1,
        std::bind(&ICP_node::raw_cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to cloud_topic: %s, to use another topic use the "
                "argument: '--ros-args -p cloud_topic:=/another/topic'",
                cloud_topic.c_str());

    pub_cloud_original_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "icp/original_cloud", 1);
    pub_cloud_transfomred_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "icp/transformed_cloud", 1);
    pub_algigned_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "icp/algined_cloud", 1);
    RCLCPP_INFO(this->get_logger(), "Node intialized sucessfully.");
}

void ICP_node::raw_cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr raw_cloud_msg) {

    PCLPointCloud::Ptr pclCloudOriginal(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudTransformed(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudAligned(new PCLPointCloud);

    // Convert raw cloud to pcl-cloud, and downsample to lower ICP run-time
    pcl::fromROSMsg(*raw_cloud_msg, *pclCloudOriginal);
    pclCloudOriginal = downSample(pclCloudOriginal);

    // Transform the cloud to have something to do the ICM on
    Eigen::Matrix4d transformationMatrix =
        getFullTransformationMatrix(0, 0, M_PI / 10, 0, 0.1, 0);

    std::string matrixPrint = Matrix4dToStr(transformationMatrix);
    RCLCPP_INFO(
        this->get_logger(),
        "Transforming the incomming point cloud with the transformation:\n%s",
        matrixPrint.c_str());

    pcl::transformPointCloud(*pclCloudOriginal, *pclCloudTransformed,
                             transformationMatrix);

    *pclCloudAligned = *pclCloudTransformed;

    // Align the point cloud with ICP
    RCLCPP_INFO(this->get_logger(),
                "Applying ICM transformed->original (%d points)",
                pclCloudOriginal->size());

    bool ICM_status = applyICM(pclCloudAligned, pclCloudOriginal);
    if (ICM_status){
        RCLCPP_INFO(this->get_logger(), "Cloud converged.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Cloud did not converge.");
    }

    // Convert transformed point cloud to rosmsg
    sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*pclCloudTransformed, transformed_cloud_msg);

    // Convert aligned point cloud to rosmsg
    sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
    pcl::toROSMsg(*pclCloudAligned, aligned_cloud_msg);

    // Publish all clouds at the same time for comparing
    this->pub_cloud_original_->publish(*raw_cloud_msg);
    this->pub_cloud_transfomred_->publish(transformed_cloud_msg);
    this->pub_algigned_cloud_->publish(aligned_cloud_msg);
}
