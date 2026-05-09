#pragma once

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"

// Headers for sysfs PWM
#include <fcntl.h>      // For open()
#include <unistd.h>     // For close(), pwrite()
#include <fstream>
#include <thread>
#include <chrono>

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
    // These hold the raw numerical data for ALL joints dynamically.
    std::vector<double> hw_states_;
    std::vector<double> hw_commands_;

    // --- The Real-Time Execution Map ---
    // This struct is highly optimized for the write/read loops.
    // It maps a physical PWM channel directly to its memory index.
    struct MotorTarget {
        int pwm_channel;
        size_t cmd_index;   // Where to find the command in hw_commands_
        int duty_cycle_fd = -1;
        int in1_gpio = -1;              // BCM GPIO line offset for IN1 (L298N direction)
        int in2_gpio = -1;              // BCM GPIO line offset for IN2 (L298N direction)
        struct gpiod_line* in1_line = nullptr;  // libgpiod line handle for IN1
        struct gpiod_line* in2_line = nullptr;  // libgpiod line handle for IN2
    };

    struct StateUpdater {
        ssize_t cmd_pos_idx = -1;
        ssize_t cmd_vel_idx = -1;
        ssize_t state_pos_idx = -1;
        ssize_t state_vel_idx = -1;
    };
    std::vector<StateUpdater> state_updaters_;

    // Only stores joints that actively receive commands (ignores passive joints like rear wheels)
    std::vector<MotorTarget> active_motors_;

    bool write_sysfs(const std::string& path, const std::string& value);

    // libgpiod chip handle (shared across all motors)
    std::string gpio_chip_name_;                // e.g. "gpiochip4" on Pi 5
    struct gpiod_chip* gpio_chip_ = nullptr;

    // Persistent clock for throttled logging — avoids dangling pointer from temporaries
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

} 