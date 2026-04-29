#include "motor_driver.hpp"
#include <cmath>

namespace motor_driver {

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
    this->pwm_fd_ = open(this->device_path_.c_str(), O_RDWR);
    if (this->pwm_fd_ < 0) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorDriver::on_deactivate(const rclcpp_lifecycle::State &) {
    if (this->pwm_fd_ >= 0) {
        close(this->pwm_fd_);
        this->pwm_fd_ = -1;
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
    // Loop ONLY over motors with a valid PWM channel.
    for (const auto& motor : this->active_motors_) {
        struct pwm_state state;
        state.period = 20000000; 

        // Direct O(1) memory lookup for the command value
        double target_cmd = this->hw_commands_[motor.cmd_index];
        
        state.duty_cycle = static_cast<uint64_t>(std::abs(target_cmd) * 2000000.0); 
        state.polarity = PWM_POLARITY_NORMAL;
        state.enabled = (state.duty_cycle > 0);

        if (ioctl(this->pwm_fd_, PWM_SET_STATE(motor.pwm_channel), &state) < 0) {
            // Depending on strictness, you can return ERROR or just print a warning
        }   
    }

    return hardware_interface::return_type::OK;
}

} 