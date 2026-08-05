#pragma once
#include "telemetry_data/telemetry_sender.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
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

        void team_occupied_callback(const std_msgs::msg::Bool::SharedPtr msg, int team_id);

        std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> teams_sub_;
        TelemetrySender telemetry_sender_;
};
