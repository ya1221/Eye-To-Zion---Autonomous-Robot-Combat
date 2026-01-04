#include "robot_description/TwistToAckermann.hpp"

TwistToStamped::TwistToStamped()
: rclcpp::Node("twist_to_stamped")
{
  using std::placeholders::_1;

  sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&TwistToStamped::twistCallback, this, _1));

  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "ackermann_steering_controller/reference", 10);

  RCLCPP_INFO(this->get_logger(), "TwistToStamped node started");
}

void TwistToStamped::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = this->now();
  out.header.frame_id = "base_link";  // or whatever you need
  out.twist = *msg;
  pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToStamped>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
