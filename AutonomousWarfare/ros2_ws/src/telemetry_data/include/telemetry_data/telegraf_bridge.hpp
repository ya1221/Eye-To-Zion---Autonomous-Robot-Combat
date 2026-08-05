#pragma once
#include "telemetry_data/telemetry_sender.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>

class TelegrafBridge : public rclcpp::Node
{
    using Clock = std::chrono::system_clock;
    using Nanoseconds = std::chrono::nanoseconds;

    public:
        TelegrafBridge();

    private:
        int64_t get_timestamp();

        // --- Callbacks ---
        void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void ammo_callback(const std_msgs::msg::Int32::SharedPtr msg);
        void health_callback(const std_msgs::msg::Int32::SharedPtr msg);

        // --- Subscribers ---
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ammo_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr health_sub_;

        TelemetrySender telemetry_sender_;
};
