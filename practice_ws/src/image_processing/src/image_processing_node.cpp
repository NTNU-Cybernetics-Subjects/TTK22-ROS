#include <ros/ros.h>
#include "image_processing/image_processing.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processing");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ImageProcessing ImageProcessing(nh, private_nh);
	ros::spin();
	return 0;
}
