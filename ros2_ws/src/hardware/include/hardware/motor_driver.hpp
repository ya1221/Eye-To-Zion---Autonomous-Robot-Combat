#pragma once

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"

// NEW HEADERS for sysfs
#include <fcntl.h>      // For open()
#include <unistd.h>     // For close(), pwrite()
#include <fstream>
#include <thread>
#include <chrono>

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
        size_t state_index; // Where to mirror the state in hw_states_ (for open-loop)
        int duty_cycle_fd = -1;
        int in1_gpio = -1;     // BCM GPIO number for IN1 (L298N direction)
        int in2_gpio = -1;     // BCM GPIO number for IN2 (L298N direction)
        int in1_value_fd = -1; // Cached fd for real-time safe GPIO writes
        int in2_value_fd = -1;
    };

    // Only stores joints that actively receive commands (ignores passive joints like rear wheels)
    std::vector<MotorTarget> active_motors_;


    bool write_sysfs(const std::string& path, const std::string& value);
    int setup_gpio(int bcm_gpio);
    void cleanup_gpio(int& fd, int bcm_gpio);

    int gpio_chip_base_ = 0;  // sysfs GPIO base offset (Pi 5 RP1 = ~571)

    // Persistent clock for throttled logging — avoids dangling pointer from temporaries
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

} 