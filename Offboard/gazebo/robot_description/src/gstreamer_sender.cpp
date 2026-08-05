#include "robot_description/gstreamer_sender.hpp"

GStreamerSender::GStreamerSender() : Node("gstreamer_sender_node") {    
    // Open the GStreamer pipeline
    writer_.open(pipeline_, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(1280, 720), true);
    
    // check if the pipeline is opened successfully
    if (!writer_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline!");
    }

    // create a subscription to the topic /camera/image
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&GStreamerSender::image_callback, this, std::placeholders::_1)
    );
}

void GStreamerSender::image_callback(const sensor_msgs::msg::Image::SharedPtr image) {
    // convert the image from ROS to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // write the image to GStreamer pipeline if it is opened
    if (writer_.isOpened()) {
        writer_.write(cv_ptr->image);
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GStreamerSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}