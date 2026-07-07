#include "hardware/twist_to_ackermann.hpp"
#include <cmath>

using std::placeholders::_1;

TwistToAckermann::TwistToAckermann(): rclcpp::Node("twist_to_ackermann")
{
  // Declare a parameter for the offset using the default value from the header file
  this->declare_parameter<double>("steering_angle_offset", default_steering_angle_offset_);

  sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&TwistToAckermann::twistCallback, this, _1));

  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "ackermann_steering_controller/reference", 10);

  RCLCPP_INFO(this->get_logger(), "TwistToAckermann node started");
}


void TwistToAckermann::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::unique_ptr<geometry_msgs::msg::TwistStamped> out = std::make_unique<geometry_msgs::msg::TwistStamped>();
  out->header.stamp = this->now();
  out->header.frame_id = "base_link";
  out->twist = *msg;

  // Read the offset parameter (in degrees) and convert to radians
  double offset_deg = this->get_parameter("steering_angle_offset").as_double();
  double offset_rad = offset_deg * (M_PI / 180.0);
  
  // Note: angular.z is angular velocity (rad/s), not steering angle directly.
  // Adding an offset here means adding a constant angular velocity.
  out->twist.angular.z += offset_rad;

  RCLCPP_INFO(this->get_logger(), "The angle is: %f", out->twist.angular.z);
  pub_->publish(std::move(out));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToAckermann>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}