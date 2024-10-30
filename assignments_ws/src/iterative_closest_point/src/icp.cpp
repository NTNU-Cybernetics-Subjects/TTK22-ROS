#include "iterative_closest_point/icp.h"
#include "Eigen/src/Geometry/AngleAxis.h"
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

ICP::ICP(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private){

    this->nh_ = nh;
    this->nh_private_ = nh_private;

    this->pub_cloud_transform_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/icp/cloud_transformed", 1);
    this->pub_cloud_original_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/icp/cloud_original", 1);
    this->sub_ = nh_private_.subscribe("/os_cloud_node/points", 1, &ICP::callback, this);

    ROS_INFO("Initializing ICP node");

    return;
}

void ICP::callback(const sensor_msgs::PointCloud2ConstPtr &msgCloud){

    PCLPointCloud::Ptr pclCloudRaw(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudTransformed(new PCLPointCloud);
    pcl::fromROSMsg(*msgCloud, *pclCloudRaw);

    // Generate transformation matrix
    Eigen::Matrix4d transformationMatrix = getFullTransformationMatrix(0, 0, M_PI/4,
                                                                       0, 0.5, 0);
    
    // Apply ICM transformation
    ROS_INFO("Applying ICM on pointcloud.");
    rosPrintMatrix4dInfo(transformationMatrix);
    bool ICM_status = this->applyICM(pclCloudRaw, pclCloudTransformed, transformationMatrix);

    // Convert transfomred point cloud to rosmsg
    sensor_msgs::PointCloud2 msgTransformedCloud;
    pcl::toROSMsg(*pclCloudTransformed, msgTransformedCloud);

    this->pub_cloud_transform_.publish(msgTransformedCloud);
    this->pub_cloud_original_.publish(msgCloud); // Publish the original pointcloud at the same time to compare (because of delay)
}

bool ICP::applyICM(PCLPointCloud::Ptr &pclCloudIn,
                   PCLPointCloud::Ptr &pclCloudOut,
                   Eigen::Matrix4d &transformationMatrix){

    pcl:pcl::console::TicToc time;

    pcl::transformPointCloud(*pclCloudIn,
                             *pclCloudOut,
                             transformationMatrix);

    int iterations = 1;

    time.tic();
    pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
    icp.setMaximumIterations(iterations);

    icp.setInputSource(pclCloudOut);
    icp.setInputTarget(pclCloudIn);

    icp.align(*pclCloudOut);

    ROS_INFO("Applied %d iterations, elapsed time: %.2f", iterations , time.toc());

    if (icp.hasConverged()){
        ROS_INFO("ICP has converged, score is %.2f", icp.getFitnessScore());
        return 0;
    }
    else {
        ROS_ERROR("ICP has not converged.");
        return 1;
    }
}

