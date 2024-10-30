cmake_minimum_required(VERSION 3.0.2)
project(image_processing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS core highgui)

catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

include_directories(
    include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OpenCV_LIBRARIES}
)

add_executable(image_processing_node src/image_processing_node.cpp src/image_processing.cpp)
target_link_libraries(image_processing_node ${OpenCV_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${catkin_LIBRARIES})


