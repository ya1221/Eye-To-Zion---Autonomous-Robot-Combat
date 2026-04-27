#include "hardware/motor_driver.hpp"

namespace motor_driver{
    MotorDriver::MotorDriver() : hardware_interface::ActuatorInterface{
        
    }


    hardware_interface::CallbackReturn MotorDriver::on_init(const hardware_interface::HardwareInfo & info) override{
        if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) 
            return CallbackReturn::ERROR;

        this->device_path_ = this->info_.hardware_parameters["pwm_device"];
        this->pwm_channel_ = std::stoi(this->info_.hardware_parameters["pwm_channel"]);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override{
        this->pwm_fd_ = open(this->device_path_.c_str(), O_RDWR);
        if (this->pwm_fd_ < 0) 
            return hardware_interface::CallbackReturn::ERROR;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override{
        if (this->pwm_fd_ >= 0) {
            close(this->pwm_fd_);
            this->pwm_fd_ = -1;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>  MotorDriver::export_state_interfaces() override{

    }

    std::vector<hardware_interface::CommandInterface>  MotorDriver::export_command_interfaces() override{

    }

    hardware_interface::return_type  MotorDriver::read(const rclcpp::Time & time, const rclcpp::Duration & period) override{
        
    }

    hardware_interface::return_type MotorDriver::write(const rclcpp::Time & time, const rclcpp::Duration & period) override{
        struct pwm_state state;

        // 1. Set the period (e.g., 20ms for 50Hz = 20,000,000 ns)
        state.period = 20000000; 

        // 2. Calculate duty cycle based on command
        // Example: mapping 0-10 rad/s to 0-period
        state.duty_cycle = static_cast<uint64_t>(std::abs(this->hw_command_) * 2000000);
        
        // 3. Set polarity and enable
        state.polarity = PWM_POLARITY_NORMAL;
        state.enabled = (state.duty_cycle > 0);

        // 4. Send to RP1 controller via ioctl
        // Note: Some drivers use PWM_SET_STATE; others use channel-specific offsets
        if (ioctl(this->pwm_fd_, PWM_SET_STATE(this->pwm_channel_), &state) < 0) {
            return hardware_interface::return_type::ERROR;
        }   

        // 5. (Optional) Update your L298N IN1/IN2 GPIO pins here for direction
        return hardware_interface::return_type::OK;
    }

}