#include "iterative_closest_point/icp.h"
#include "ros/console.h"

#include <cmath>
#include <sensor_msgs/PointCloud2.h>

#include <string>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
// #include <pcl_conversions/pcl_conversions.h>


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

    // ROS_INFO("Recived pointcloud. Passthrough");
    PCLPointCloud::Ptr pclCloudRaw(new PCLPointCloud);
    PCLPointCloud::Ptr pclCloudTransformed(new PCLPointCloud);
    pcl::fromROSMsg(*msgCloud, *pclCloudRaw);
    bool ICM_status = this->applyICM(pclCloudRaw, pclCloudTransformed); // Test 
    // ROS_INFO("ICM transformation was %d", ICM_status);
    // *pclCloudTransformed = *pclCloudRaw;

    // Publish transformed pointcloud

    sensor_msgs::PointCloud2 msgTransformedCloud;
    pcl::toROSMsg(*pclCloudTransformed, msgTransformedCloud);
    this->pub_cloud_transform_.publish(msgTransformedCloud);
    this->pub_cloud_original_.publish(msgCloud); // publish the original at the same time to compare (because of delay)
}

void print4x4Matrix(const Eigen::Matrix4d &matrix){
    ROS_INFO("Rotation matrix :\n");
    ROS_INFO("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    ROS_INFO("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    ROS_INFO("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    ROS_INFO("Translation vector :\n");
    ROS_INFO("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}


// Execute ICP algorithm
bool ICP::applyICM(PCLPointCloud::Ptr &pclCloudIn, PCLPointCloud::Ptr &pclCloudOut){

    pcl:pcl::console::TicToc time;

    // PCLPointCloud::Ptr pclCloudTr(new PCLPointCloud);

    // pcl::PCLPointCloud2::Ptr pclCloudTr (new pcl::PCLPointCloud2());
    // pcl::PCLPointCloud2::Ptr pclCloudOut (new pcl::PCLPointCloud2());

    // Transformation matrix
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    // Rotation
    double theta = M_PI / 8; // angle of rotation
    transformationMatrix(0,0) = std::cos(theta);
    transformationMatrix(0,1) = -std::sin(theta);
    transformationMatrix(1,0) = std::sin(theta);
    transformationMatrix(1,1) = std::cos(theta);
    // Translation
    // transformationMatrix(0,3) = 0.4; // x
    transformationMatrix(1,3) = 0.4; // y
    // transformationMatrix(2,3) = 0.4; // z

    pcl::transformPointCloud(*pclCloudIn, *pclCloudOut, transformationMatrix);
    // print4x4Matrix(transformationMatrix);
    // *pclCloudTr = *pclCloudOut;

    int iterations = 1;
    time.tic();
    pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(pclCloudOut);
    icp.setInputTarget(pclCloudIn);
    icp.align(*pclCloudOut);
    ROS_INFO("Applied %d iterations, elapsed time: %.2f", iterations ,time.toc());

    if (icp.hasConverged()){
        ROS_INFO("ICP has converged, score is %.2f", icp.getFitnessScore());
        // transformationMatrix = icp.getFinalTransformation().cast<double>();
        return 0;
    }
    else {
        ROS_ERROR("ICP has not converged.");
        return 1;
    }
}