#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string>

class ShootingNode : public rclcpp::Node {
public:
    ShootingNode();

    // Call this from your trigger input (button, joystick callback, etc.)
    void set_firing(bool on);

private:
    void tick();
    void publish_state();

    // Subscription callback — receives mode changes ("single" / "auto")
    // from an external node (e.g. main_brain) on /shooting_mode.
    void mode_callback(const std_msgs::msg::String::SharedPtr msg);

    // ROS2 dynamic-parameter callback — validates runtime changes to
    // fire_rate_hz (blocked while in single mode).
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(
        const std::vector<rclcpp::Parameter>& params);

    // Service callback for the ~/fire_once single-shot trigger.
    void fire_once_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_toggle_time_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fire_once_srv_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    bool state_ = false;
    bool firing_ = true;

    // Current fire mode — updated by the /shooting_mode subscriber.
    std::string fire_mode_ = "auto";

    // Single-shot support: when true, the next tick produces exactly one
    // HIGH→LOW pulse and then clears the flag.
    bool single_shot_pending_ = false;

    // Internal flag: allows mode_callback to reset fire_rate_hz even while
    // in single mode (the on_parameters_changed callback normally blocks it).
    bool internal_rate_reset_ = false;
};