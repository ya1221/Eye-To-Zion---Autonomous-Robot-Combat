#include "hardware/motor_driver.hpp"
#include <cmath>
#include <iostream>
#include <cerrno>
#include <cstring>
#include "rclcpp/rclcpp.hpp"

namespace motor_driver {


bool MotorDriver::write_sysfs(const std::string& path, const std::string& value) {
    int fd = open(path.c_str(), O_WRONLY);
    if (fd < 0) return false;
    ::write(fd, value.c_str(), value.length());
    close(fd);
    return true;
}



hardware_interface::CallbackReturn MotorDriver::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->device_path_ = this->info_.hardware_parameters.at("pwm_device");

    size_t total_states = 0;
    size_t total_commands = 0;

    // Dynamically map memory based on whatever is inside the URDF
    for (const auto & joint : info_.joints) {
        
        // 1. Check if this joint has a PWM channel assigned in the URDF
        int pwm_channel = -1;
        if (joint.parameters.find("pwm_channel") != joint.parameters.end()) {
            pwm_channel = std::stoi(joint.parameters.at("pwm_channel"));
        }

        // 2. If it has a channel AND accepts commands, add it to the active motors list
        if (pwm_channel != -1 && !joint.command_interfaces.empty()) {
            MotorTarget target;
            target.pwm_channel = pwm_channel;
            
            // The index will just be the current total_commands size
            target.cmd_index = total_commands; 
            target.state_index = total_states; 
            
            this->active_motors_.push_back(target);
        }

        // 3. Keep counting to allocate enough memory for everything
        total_states += joint.state_interfaces.size();
        total_commands += joint.command_interfaces.size();
    }

    // Allocate memory ONCE. The pointers will remain stable forever.
    this->hw_states_.resize(total_states, 0.0);
    this->hw_commands_.resize(total_commands, 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorDriver::on_activate(const rclcpp_lifecycle::State &) {
    for (auto & motor : this->active_motors_) {
        // 1. Export the channel
        std::string export_path = this->device_path_ + "/export";
        write_sysfs(export_path, std::to_string(motor.pwm_channel));

        // Wait slightly: sysfs can take a few milliseconds to generate the new directory
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::string pwm_channel_dir = this->device_path_ + "/pwm" + std::to_string(motor.pwm_channel);

        // 2. Set the Period (e.g., 20,000,000 ns = 50Hz)
        write_sysfs(pwm_channel_dir + "/period", "20000000");

        // 3. Enable the PWM
        write_sysfs(pwm_channel_dir + "/enable", "1");

        // 4. Open and cache the duty_cycle file descriptor for the real-time write loop
        std::string duty_path = pwm_channel_dir + "/duty_cycle";
        motor.duty_cycle_fd = open(duty_path.c_str(), O_WRONLY);
        
        if (motor.duty_cycle_fd < 0) {
            // Fails if docker wasn't run in privileged mode or volume wasn't mounted
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorDriver::on_deactivate(const rclcpp_lifecycle::State &) {
    for (auto & motor : this->active_motors_) {
        if (motor.duty_cycle_fd >= 0) {
            // Safety: Set duty cycle to 0 before closing
            pwrite(motor.duty_cycle_fd, "0", 1, 0);
            close(motor.duty_cycle_fd);
            motor.duty_cycle_fd = -1;
        }

        std::string pwm_channel_dir = this->device_path_ + "/pwm" + std::to_string(motor.pwm_channel);
        
        // Disable the channel
        write_sysfs(pwm_channel_dir + "/enable", "0");
        
        // Unexport the channel to clean up sysfs
        std::string unexport_path = this->device_path_ + "/unexport";
        write_sysfs(unexport_path, std::to_string(motor.pwm_channel));
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorDriver::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    size_t global_index = 0;
    
    // Completely general: Exports whatever state tags exist in the URDF
    for (const auto& joint : this->info_.joints) {
        for (const auto& interface : joint.state_interfaces) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                joint.name, interface.name, &this->hw_states_[global_index++]));
        }
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorDriver::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    size_t global_index = 0;
    
    // Completely general: Exports whatever command tags exist in the URDF
    for (const auto& joint : this->info_.joints) {
        for (const auto& interface : joint.command_interfaces) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                joint.name, interface.name, &this->hw_commands_[global_index++]));
        }
    }
    return command_interfaces;
}

hardware_interface::return_type MotorDriver::read(const rclcpp::Time &, const rclcpp::Duration &) {
    // Open-Loop mapping: simply mirror the command values to state values.
    // Notice how fast this loop is—no strings, no nested loops over inactive joints.
    for (const auto& motor : this->active_motors_) {
        this->hw_states_[motor.state_index] = this->hw_commands_[motor.cmd_index];
    }
    
    // Note: Joints with no commands (rear wheels) remain 0.0 
    // until you connect physical encoders and write their data here.
    return hardware_interface::return_type::OK; 
}

hardware_interface::return_type MotorDriver::write(const rclcpp::Time &, const rclcpp::Duration &) {
    for (const auto& motor : this->active_motors_) {
        if (motor.duty_cycle_fd < 0) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 2000,
                "PWM channel %d: fd is invalid (%d)", motor.pwm_channel, motor.duty_cycle_fd);
            continue;
        }

        // Calculate the duty cycle in nanoseconds
        double target_cmd = this->hw_commands_[motor.cmd_index];
        uint64_t duty_cycle_ns = static_cast<uint64_t>(std::abs(target_cmd) * 2000000.0);

        // Clamp to period (20ms = 20,000,000 ns)
        if (duty_cycle_ns > 20000000) {
            duty_cycle_ns = 20000000;
        }

        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 2000,
            "PWM ch%d: cmd_index=%zu, target_cmd=%.4f, duty_cycle_ns=%lu, fd=%d",
            motor.pwm_channel, motor.cmd_index, target_cmd, duty_cycle_ns, motor.duty_cycle_fd);

        // REAL-TIME SAFE CONVERSION:
        // We use snprintf instead of std::to_string() to avoid heap memory allocations in the RT loop
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "%lu", duty_cycle_ns);

        // REAL-TIME SAFE WRITE:
        // We use `pwrite` instead of `write`. Normal `write` advances the file cursor. 
        // `pwrite(..., 0)` forces the kernel to overwrite from the very beginning of the file every time.
        ssize_t ret = pwrite(motor.duty_cycle_fd, buf, len, 0);
        if (ret < 0) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 2000,
                "PWM ch%d: pwrite failed! errno=%d (%s), buf='%s'",
                motor.pwm_channel, errno, std::strerror(errno), buf);
        }
    }

    return hardware_interface::return_type::OK;
}

} 


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motor_driver::MotorDriver, 
  hardware_interface::SystemInterface
)