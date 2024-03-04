#ifndef __RECIEVE_IMAGE_NODE_HPP__
#define __RECIEVE_IMAGE_NODE_HPP__

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

#define OUTPUT_IMAGE_WIDTH 1920
#define OUTPUT_IMAGE_HEIGHT 1080

class RecieveTopicNode : public rclcpp::Node
{
public:
    RecieveTopicNode()
        : Node("camera_display_node")
    {
        camera_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "image",
            rclcpp::SensorDataQoS(), std::bind(&RecieveTopicNode::getCameraImage, this, std::placeholders::_1));
    }

    ~RecieveTopicNode()
    {
    }

    cv::Mat camera_image_;

private:
    void getCameraImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image message to OpenCV image
            camera_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
};

#endif //__RECIEVE_IMAGE_NODE_HPP__
