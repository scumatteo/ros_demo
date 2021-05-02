#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>
#include <chrono>

void onBGRMessage(const sensor_msgs::ImageConstPtr &bgr)
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cv_bridge::CvImagePtr cv_bgr_ptr;

    try
    {

        //create raw image
        cv_bgr_ptr = cv_bridge::toCvCopy(bgr, sensor_msgs::image_encodings::BGR8); //convert to CvImage
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge RGB exception: %s", e.what());
        return;
    }

    ROS_INFO("rgb no sync time: %f", cv_bgr_ptr->header.stamp.toSec());
}

void onDepthMessage(const sensor_msgs::ImageConstPtr &depth)
{
    cv_bridge::CvImagePtr cv_depth_ptr;

    try
    {

        //create aligned depth image
        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1); //convert to CvImage
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge ALIGN DEPTH exception: %s", e.what());
        return;
    }

    ROS_INFO("depth no sync time: %f", cv_depth_ptr->header.stamp.toSec());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_no_sync_subscriber_node");
    ros::NodeHandle nh;

    const int QUEUE_SIZE = 1;

    ros::Subscriber bgr_subscriber = nh.subscribe("/camera/color/image_raw", QUEUE_SIZE, onBGRMessage);
    ros::Subscriber depth_subscriber = nh.subscribe("/camera/depth/image_rect_raw", QUEUE_SIZE, onDepthMessage);
    //ros::MultiThreadedSpinner();
    ros::spin();

    return 0;
}