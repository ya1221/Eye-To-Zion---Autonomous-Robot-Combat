#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class ShootingNode : public rclcpp::Node {
public:
    ShootingNode();

    // Call this from your trigger input (button, joystick callback, etc.)
    void set_firing(bool on);

private:
    void tick();
    void publish_state();

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_toggle_time_;
    bool state_ = false;
    bool firing_ = true;
};