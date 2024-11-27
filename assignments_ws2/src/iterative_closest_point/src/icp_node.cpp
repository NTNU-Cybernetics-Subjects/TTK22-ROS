#include "iterative_closest_point/icp.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// #include <functional>
// #include <memory>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/utilities.hpp>

ICP_node::ICP_node() : Node("icp_node") {

    this->sub_raw_cloud_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/os_cloud_node/points", 1,
            std::bind(&ICP_node::raw_cloud_callback, this,
                      std::placeholders::_1));

    this->pub_cloud_original_ =
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
    RCLCPP_INFO(
        this->get_logger(),
        "Transforming the incomming point cloud with the transformation:");
    rosPrintMatrix4dInfo(transformationMatrix);
    pcl::transformPointCloud(*pclCloudOriginal, *pclCloudTransformed,
                             transformationMatrix);

    *pclCloudAligned = *pclCloudTransformed;

    // Align the point cloud with ICP
    RCLCPP_INFO(this->get_logger(),
                "Applying ICM transformed->original with size = %d",
                pclCloudOriginal->size());
    bool ICM_status = applyICM(pclCloudAligned, pclCloudOriginal);

    // Convert transformed point cloud to rosmsg
    sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*pclCloudTransformed, transformed_cloud_msg);
    // msgTransformedCloud.header.stamp = ros::Time::now();

    // Convert aligned point cloud to rosmsg
    sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
    pcl::toROSMsg(*pclCloudAligned, aligned_cloud_msg);
    // msgAlignedCloud.header.stamp = ros::Time::now();

    // Publish all clouds at the same time for comparing
    this->pub_cloud_original_->publish(*raw_cloud_msg);
    this->pub_cloud_transfomred_->publish(transformed_cloud_msg);
    this->pub_algigned_cloud_->publish(aligned_cloud_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP_node>());
    rclcpp::shutdown();
    return 0;
}
