#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>
#include <chrono>

/**
 * Camera policy to receive rgb and depth images with the same timestamp.
 */
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> RGBDCameraSyncPolicy;

class ImageSubscriberNodeCpp
{
private:
    const int _QUEUE_SIZE = 1;

    //raw and depth images subscribers
    message_filters::Subscriber<sensor_msgs::Image> _raw_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> _align_depth_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_image_sub;

    //synchronizer for raw and depth images
    message_filters::Synchronizer<RGBDCameraSyncPolicy> _camera_sync;

    ros::Publisher _publisher;

public:
    ros::NodeHandle nh;

    ImageSubscriberNodeCpp() : _camera_sync(RGBDCameraSyncPolicy(this->_QUEUE_SIZE),
                                            this->_raw_image_sub,
                                            this->_align_depth_image_sub,
                                            this->_depth_image_sub)
    {
        //topic subscription
        this->_raw_image_sub.subscribe(nh, "/camera/color/image_raw", this->_QUEUE_SIZE);
        this->_align_depth_image_sub.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", this->_QUEUE_SIZE);
        this->_depth_image_sub.subscribe(nh, "/camera/depth/image_rect_raw", this->_QUEUE_SIZE);

        //registration of sync callbac
        this->_camera_sync.registerCallback(boost::bind(&ImageSubscriberNodeCpp::onCameraMessage, this, _1, _2, _3));

        this->_publisher = nh.advertise<sensor_msgs::Image>("/camera/grayscale/image_raw", this->_QUEUE_SIZE);
    }

    void onCameraMessage(const sensor_msgs::ImageConstPtr &bgr,
                         const sensor_msgs::ImageConstPtr &align_depth,
                         const sensor_msgs::ImageConstPtr &depth)
    {

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        cv_bridge::CvImagePtr cv_bgr_ptr;
        cv_bridge::CvImagePtr cv_align_depth_ptr;
        cv_bridge::CvImagePtr cv_depth_ptr;

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
        try
        {

            //create aligned depth image
            cv_align_depth_ptr = cv_bridge::toCvCopy(align_depth, sensor_msgs::image_encodings::TYPE_16UC1); //convert to CvImage
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge ALIGN DEPTH exception: %s", e.what());
            return;
        }
        try
        {

            //create depth image
            cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1); //convert to CvImage
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge DEPTH exception: %s", e.what());
            return;
        }

        cv::Mat bgr_image = cv_bgr_ptr->image;
        cv::Mat align_depth_image = cv_align_depth_ptr->image;
        cv::Mat depth_image = cv_depth_ptr->image;

        ROS_INFO("rgb sync time: %f", cv_bgr_ptr->header.stamp.toSec());
        ROS_INFO("aligned depth sync time: %f", cv_align_depth_ptr->header.stamp.toSec());
        ROS_INFO("depth sync time: %f", cv_depth_ptr->header.stamp.toSec());

        cv::Mat gray_image;
        cv::cvtColor(bgr_image, gray_image, CV_BGR2GRAY);

        /**
         * Conversion from cv::Mat to sensor_msgs::Image
         */
        cv_bridge::CvImage gray_image_msg;
        gray_image_msg.encoding = sensor_msgs::image_encodings::TYPE_8SC1;
        gray_image_msg.image = gray_image;

        this->_publisher.publish(gray_image_msg.toImageMsg());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_subscriber_node_cpp");
    ImageSubscriberNodeCpp image_sub;
    ros::spin();
    return 0;
}