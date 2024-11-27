#include "iterative_closest_point/icp.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP_node>());
    rclcpp::shutdown();
    return 0;
}
