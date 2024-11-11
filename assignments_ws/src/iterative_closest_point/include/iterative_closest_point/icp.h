#ifndef ICP_H
#define ICP_H

#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;

class ICP {

    public:
    ICP(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    bool applyICM(PCLPointCloud::Ptr &pclCloudSource, PCLPointCloud::Ptr &pclCloudTarget);
    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud);

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_;
    ros::Publisher pub_cloud_transform_;
    ros::Publisher pub_cloud_original_;
};


PCLPointCloud::Ptr downSample(PCLPointCloud::Ptr &raw_cloud);

Eigen::Matrix4d getFullTransformationMatrix(double roll, double pitch, double yaw,
                                        double xTrans, double yTrans, double zTrans);

void rosPrintMatrix4dInfo(Eigen::Matrix4d &matrix);

#endif // !ICP_H

