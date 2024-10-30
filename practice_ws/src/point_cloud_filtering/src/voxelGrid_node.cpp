#include "point_cloud_filtering/point_cloud_filtering.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "voxel_filter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    VoxelGrid voxelGrid_node(nh, private_nh);
    ros::spin();
    return 0;
}
