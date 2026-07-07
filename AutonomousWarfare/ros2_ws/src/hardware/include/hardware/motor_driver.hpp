#pragma once

#include <vector>
#include <string>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/bool.hpp"

// Headers for sysfs PWM
#include <fcntl.h>      // For open()
#include <unistd.h>     // For close(), pwrite()
#include <fstream>
#include <thread>
#include <chrono>

// Serial port configuration
#include <termios.h>

// libgpiod for L298N direction pins
#include <gpiod.h>

namespace motor_driver {

class MotorDriver : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MotorDriver)

    // Lifecycle Methods
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
    int pwm_fd_ = -1;
    std::string device_path_;

    // --- The Flat Memory Buffers ---
    std::vector<double> hw_states_;
    std::vector<double> hw_commands_;

    // --- The Real-Time Execution Map ---
    struct MotorTarget {
        int pwm_channel;
        size_t cmd_index;
        int duty_cycle_fd = -1;
        int in1_gpio = -1;
        int in2_gpio = -1;
        struct gpiod_line* in1_line = nullptr;
        struct gpiod_line* in2_line = nullptr;
    };

    // --- Steering Servo Target ---
    struct ServoTarget {
        int servo_index;
        size_t cmd_index;
    };

    struct StateUpdater {
        ssize_t cmd_pos_idx = -1;
        ssize_t cmd_vel_idx = -1;
        ssize_t state_pos_idx = -1;
        ssize_t state_vel_idx = -1;
    };

    std::vector<StateUpdater> state_updaters_;
    std::vector<MotorTarget> active_motors_;
    std::vector<ServoTarget> active_servos_;

    bool write_sysfs(const std::string& path, const std::string& value);

    // --- Serial (UART) for Arduino ---
    std::string serial_device_path_;
    int serial_baud_ = 115200;
    int serial_fd_ = -1;

    // --- PWM multiplier ---
    double pwm_multiplier_ = 2.0;

    bool open_serial_port();

    // libgpiod chip handle
    std::string gpio_chip_name_;
    struct gpiod_chip* gpio_chip_ = nullptr;

    // --- Shooting flag (received via ROS2 topic, forwarded over serial) ---
    rclcpp::Node::SharedPtr internal_node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
    std::atomic<bool> flag_active_{false};
    bool flag_last_sent_ = false;   // Tracks last state sent to Arduino to avoid redundant writes

    // Persistent clock for throttled logging
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

} // namespace motor_driver