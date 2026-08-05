#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

class RobotStats : public rclcpp::Node
{
public:
    RobotStats();

private:
    // --- Callbacks ---
    void impact_alert_callback(const std_msgs::msg::String::SharedPtr msg);
    void shooting_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // --- Subscribers ---
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr impact_alert_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shooting_cmd_sub_;

    // --- Publishers ---
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ammo_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr health_pub_;


    // --- State ---
    int ammo_;
    int health_;
};

