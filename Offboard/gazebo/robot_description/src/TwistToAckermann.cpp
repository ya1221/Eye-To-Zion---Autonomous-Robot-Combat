#include "robot_description/TwistToAckermann.hpp"

using std::placeholders::_1;

TwistToAckermann::TwistToAckermann(): rclcpp::Node("twist_to_ackermann")
{
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