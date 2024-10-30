#include "iterative_closest_point/icp.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "icp");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ICP icp_node(nh, private_nh);
    ros::spin();
    return 0;
}
