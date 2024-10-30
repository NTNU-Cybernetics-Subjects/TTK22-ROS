#include <image_processing/image_processing.h>

    ImageProcessing::ImageProcessing(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh), nh_private_(nh_private), it_(nh_private)
    {
        //Setup Publishers and Subscribers
        image_pub_ = it_.advertise("/dialated_image", 5);
        // image_sub_ = it_.subscribe("/blackfly_left/blackfly_left", 5, &ImageProcessing::dialateImage, this);
        image_sub_ = it_.subscribe("/blackfly_left/blackfly_left", 5, &ImageProcessing::bilateralFilter, this);
    }

    void ImageProcessing::dialateImage(const sensor_msgs::ImageConstPtr &msg)
    {
        std_msgs::Header header = msg->header;
        cv::Mat bgr_image = cv_bridge::toCvShare(msg, "8UC1")->image;
        cv::Mat dialated_image;
        cv::dilate(bgr_image, dialated_image, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        image_pub_.publish(cv_bridge::CvImage(header, "mono8", dialated_image).toImageMsg());
    }

    void ImageProcessing::bilateralFilter(const sensor_msgs::ImageConstPtr &msg)
    {
        std_msgs::Header header = msg->header;
        cv::Mat bgr_image = cv_bridge::toCvShare(msg, "8UC1")->image;
        cv::Mat bilateral_filtered_image;
        cv::bilateralFilter(bgr_image, bilateral_filtered_image, 9, 75, 75);
        image_pub_.publish(cv_bridge::CvImage(header, "mono8", bilateral_filtered_image).toImageMsg());
    }
