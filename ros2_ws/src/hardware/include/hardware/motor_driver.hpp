#pragma once

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include <linux/pwm.h>  // For pwm_state and ioctl commands
#include <fcntl.h>      // For open()
#include <unistd.h>     // For close()
#include <sys/ioctl.h>  // For ioctl()

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
    };

    // Only stores joints that actively receive commands (ignores passive joints like rear wheels)
    std::vector<MotorTarget> active_motors_;
};

} 