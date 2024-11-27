#include "iterative_closest_point/icp.hpp"

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
// #include <pcl_conversions/pcl_conversions.h>
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
    // ROS_INFO("%s", oss.str().c_str());
}


Eigen::Matrix4d getFullTransformationMatrix(double roll, double pitch,
                                            double yaw, double xTrans,
                                            double yTrans, double zTrans) {
    // Rotation
    Eigen::AngleAxisd zRotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yRotation =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd xRotation =
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d zyxRotationMatrix = zRotation.toRotationMatrix() *
                                        yRotation.toRotationMatrix() *
                                        xRotation.toRotationMatrix();

    // Translation
    Eigen::Vector3d translationVec{xTrans, yTrans, zTrans};

    Eigen::Matrix4d zyxtTransformationMatrix = Eigen::Matrix4d::Identity();
    zyxtTransformationMatrix.block<3, 3>(0, 0) = zyxRotationMatrix;
    zyxtTransformationMatrix.block<3, 1>(0, 3) = translationVec;
    return zyxtTransformationMatrix;
}

PCLPointCloud::Ptr downSample(PCLPointCloud::Ptr &raw_cloud) {

    PCLPointCloud::Ptr downsampled_cloud(new PCLPointCloud);

    pcl::VoxelGrid<PCLPoint> voxel_filter;
    voxel_filter.setInputCloud(raw_cloud);
    voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);
    voxel_filter.filter(*downsampled_cloud);

    return downsampled_cloud;
}

bool applyICM(PCLPointCloud::Ptr &pclCloudSource,
              PCLPointCloud::Ptr &pclCloudTarget) {

pcl:
    pcl::console::TicToc time;

    int max_iterations = 25;

    time.tic();
    pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
    icp.setMaximumIterations(max_iterations);

    icp.setInputSource(pclCloudSource);
    icp.setInputTarget(pclCloudTarget);

    icp.align(*pclCloudSource);

    // ROS_INFO("Applied %d iterations, elapsed time: %.2f", max_iterations ,
    // time.toc());

    if (icp.hasConverged()) {
        // ROS_INFO("ICP converged with score: %.2f", icp.getFitnessScore());
        return 1;
    } else {
        // ROS_ERROR("ICP did not converged.");
        return 0;
    }
}
