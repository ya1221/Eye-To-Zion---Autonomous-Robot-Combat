#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

  class TwistToAckermann : public rclcpp::Node
  {
  public:
    explicit TwistToAckermann();
  
  private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
    // Default steering angle offset; change this to your desired constant (e.g., 15.0)
    double default_steering_angle_offset_ = 0.0;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  };

