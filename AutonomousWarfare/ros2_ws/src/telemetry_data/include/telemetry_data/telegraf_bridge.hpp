#pragma once
#include "telemetry_data/telemetry_sender.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <iostream>
#include <random>

class TelegrafBridge : public rclcpp::Node
{
    using Clock = std::chrono::system_clock;
    using Nanoseconds = std::chrono::nanoseconds;

    public:
        TelegrafBridge();

    private:
        int64_t get_timestamp();
        void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
        void health_callback();
        void path_callback(const nav_msgs::msg::Path::SharedPtr path_msg);
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::mt19937 rng_;
        std::uniform_int_distribution<int> dist_;

        TelemetrySender telemetry_sender_;
};

