#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string>

class ShootingNode : public rclcpp::Node {
public:
    ShootingNode();

private:
    // Timers for each mode
    void auto_tick();
    void single_tick();
    
    void publish_cmd(bool value);

    // Subscription callback — receives mode changes ("single" / "auto")
    // from an external node (e.g. main_brain) on /shooting_mode.
    void mode_callback(const std_msgs::msg::String::SharedPtr msg);

    // Subscription callback — master enable/disable for firing.
    // true = robot is allowed to shoot, false = stop shooting immediately.
    void firing_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // ROS2 dynamic-parameter callback — validates runtime changes to
    // fire_rate_hz (blocked while in single mode).
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(
        const std::vector<rclcpp::Parameter>& params);

    // Service callback for the ~/fire_once single-shot trigger.
    // Only works when mode is "single" and firing_ is true.
    void fire_once_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr firing_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr auto_timer_;
    rclcpp::TimerBase::SharedPtr single_timer_;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fire_once_srv_;

    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // Timing for auto mode rate control
    rclcpp::Time last_shot_time_;

    // ── State ──────────────────────────────────────────────────────────────

    // Master safety switch: if false, robot cannot shoot regardless of mode.
    // Controlled via /firing topic.
    bool firing_ = false;

    // Current fire mode — updated by the /shooting_mode subscriber.
    // "single" or "auto"
    std::string fire_mode_ = "single";

    // Single-shot support: when true, the next single_tick sends exactly one
    // true command and then clears the flag.
    bool single_shot_pending_ = false;

    // Internal flag: allows mode_callback to reset fire_rate_hz even while
    // in single mode (the on_parameters_changed callback normally blocks it).
    bool internal_rate_reset_ = false;

    // Track the current command state for toggling in both modes
    bool current_cmd_state_ = false;
};