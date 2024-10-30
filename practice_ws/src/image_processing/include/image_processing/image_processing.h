#ifndef IMAGE_PROCESSING
#define IMAGE_PROCESSING


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <vector>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Header.h>

class ImageProcessing
{
public:
	ImageProcessing(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
	void dialateImage(const sensor_msgs::ImageConstPtr &msg);
	void bilateralFilter(const sensor_msgs::ImageConstPtr &msg);


private:
	// nodes
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	image_transport::ImageTransport it_;

	// publishers
	image_transport::Publisher image_pub_;
	image_transport::Subscriber image_sub_;
};


#endif
