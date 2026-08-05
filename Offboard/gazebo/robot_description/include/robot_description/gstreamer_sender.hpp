#pragma once
// this include is for using ROS 2 functions
#include <rclcpp/rclcpp.hpp>
// this include is for getting image messages from ROS
#include <sensor_msgs/msg/image.hpp>
// this include is for converting ROS image messages to OpenCV format
#include <cv_bridge/cv_bridge.h>
// this include is for using OpenCV functions
#include <opencv2/opencv.hpp>

// this class purpose is for sending images to a GStreamer pipeline from the camera
// it does this by creating a subscription to the topic /camera/image and then converting the image
// to OpenCV format and sending it to the GStreamer pipeline
class GStreamerSender : public rclcpp::Node{
    public:
        GStreamerSender();
    private:
        // subscription to the topic /camera/image
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        // writer for the GStreamer pipeline
        cv::VideoWriter writer_;
        // GStreamer pipeline and initialize the VideoWriter
        std::string pipeline_ = "appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast ! rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5000";
        // callback function that called when the subscription(image_sub_) get a new image from the topic /camera/image
        void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
};
