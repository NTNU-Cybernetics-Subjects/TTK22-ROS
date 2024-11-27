#include "iterative_closest_point/icp.h"
#include "ros/time.h"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/src/Core/Matrix.h>
#include <ostream>
#include <ros/console.h>

#include <cmath>
#include <sensor_msgs/PointCloud2.h>

#include <sstream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

void rosPrintMatrix4dInfo(Eigen::Matrix4d &matrix){
    std::ostringstream oss;
    oss << "Transformation matrix: " << std::endl << "| ";
    for (int i = 0; i<4; i++){
        for (int j = 0; j<4; j++){
            oss << "(" <<matrix(i,j) << ")   ";
        }
        oss << "  |" << std::endl;
        if (i < 3){
            oss << "|  ";
        }
    }
    ROS_INFO("%s", oss.str().c_str());
}

Eigen::Matrix4d getFullTransformationMatrix(double roll, double pitch, double yaw,
                                        double xTrans, double yTrans, double zTrans){

    // Rotation
    Eigen::AngleAxisd zRotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yRotation = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd xRotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d zyxRotationMatrix = zRotation.toRotationMatrix() * yRotation.toRotationMatrix() * xRotation.toRotationMatrix();

    // Translation
    Eigen::Vector3d translationVec{xTrans, yTrans, zTrans};

    Eigen::Matrix4d zyxtTransformationMatrix = Eigen::Matrix4d::Identity();
    zyxtTransformationMatrix.block<3, 3>(0, 0) = zyxRotationMatrix;
    zyxtTransformationMatrix.block<3, 1>(0, 3) = translationVec;
    return zyxtTransformationMatrix;
}

PCLPointCloud::Ptr downSample(PCLPointCloud::Ptr &raw_cloud){

    PCLPointCloud::Ptr downsampled_cloud(new PCLPointCloud);

	pcl::VoxelGrid<PCLPoint> voxel_filter;
	voxel_filter.setInputCloud(raw_cloud);
	voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);
	voxel_filter.filter(*downsampled_cloud);

    return downsampled_cloud;
}

ICP::ICP(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private){

    this->nh_ = nh;
    this->nh_private_ = nh_private;

    // Transformed cloud topic
    this->pub_cloud_transform_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/icp/cloud_transformed", 1);

    // Aligned cloud topic
    this->pub_cloud_aligned_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/icp/cloud_alinged", 1);

    // Original cloud topic, for compering
    this->pub_cloud_original_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/icp/cloud_original", 1);

    // incoming cloud from the bag
    std::string lider_data_topic = "/os_cloud_node/points";
    this->sub_ = nh_private_.subscribe(lider_data_topic, 1, &ICP::callback, this);

    ROS_INFO("Initializing ICP node");

    return;
}

void ICP::callback(const sensor_msgs::PointCloud2ConstPtr &msgCloud){

    PCLPointCloud::Ptr pclCloudOriginal(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudTransformed(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudAligned(new PCLPointCloud);

    // Convert raw cloud to pcl-cloud, and downsample to lower ICP run-time
    pcl::fromROSMsg(*msgCloud, *pclCloudOriginal);
    pclCloudOriginal = downSample(pclCloudOriginal);

    // Transform the cloud to have something to do the ICM on
    Eigen::Matrix4d transformationMatrix = getFullTransformationMatrix(0, 0, M_PI/10,
                                                                       0, 0.1, 0);
    ROS_INFO("Transforming the incomming point cloud with the transformation:");
    rosPrintMatrix4dInfo(transformationMatrix);
    pcl::transformPointCloud(*pclCloudOriginal, *pclCloudTransformed, transformationMatrix);

    *pclCloudAligned = *pclCloudTransformed;
    
    // Align the point cloud with ICP
    ROS_INFO("Applying ICM transformed->original with size = %d", pclCloudOriginal->size());
    bool ICM_status = this->applyICM(pclCloudAligned, pclCloudOriginal);

    // Convert transformed point cloud to rosmsg
    sensor_msgs::PointCloud2 msgTransformedCloud;
    pcl::toROSMsg(*pclCloudTransformed, msgTransformedCloud);
    msgTransformedCloud.header.stamp = ros::Time::now();

    // Convert aligned point cloud to rosmsg
    sensor_msgs::PointCloud2 msgAlignedCloud;
    pcl::toROSMsg(*pclCloudAligned, msgAlignedCloud);
    msgAlignedCloud.header.stamp = ros::Time::now();

    // Publish all clouds at the same time for comparing
    this->pub_cloud_original_.publish(msgCloud);
    this->pub_cloud_transform_.publish(msgTransformedCloud);
    this->pub_cloud_aligned_.publish(msgAlignedCloud);
}

bool ICP::applyICM(PCLPointCloud::Ptr &pclCloudSource, PCLPointCloud::Ptr &pclCloudTarget){

    pcl::console::TicToc time;

    int max_iterations = 25;

    time.tic();
    pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
    icp.setMaximumIterations(max_iterations);

    icp.setInputSource(pclCloudSource);
    icp.setInputTarget(pclCloudTarget);

    icp.align(*pclCloudSource);

    ROS_INFO("Applied %d iterations, elapsed time: %.2f", max_iterations , time.toc());

    if (icp.hasConverged()){
        ROS_INFO("ICP converged with score: %.2f", icp.getFitnessScore());
        return 1;
    }
    else {
        ROS_ERROR("ICP did not converged.");
        return 0;
    }
}

