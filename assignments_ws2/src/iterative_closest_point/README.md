# iterative_closest_point

This package implements an example of the iterative closest point algorithm in
ROS2 jazzy. The package listens to a pointcloud2 topic performs an
transformation on that cloud, and finally uses the ICP algorithm to align the
transformed cloud back to the original.

## Input cloud

The package default to use `/os_cloud_node/points` as input cloud. This can be
changed using the parameter `cloud_topic`. For example running the ICP algorithm on
another cloud:

```
ros2 run iterative_closest_point icp_node --ros-args -p
cloud_topic:=/another/topic
```


