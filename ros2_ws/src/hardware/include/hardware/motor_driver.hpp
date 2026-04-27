#pragma once

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include <linux/pwm.h>  // For pwm_state and ioctl commands
#include <fcntl.h>      // For open()
#include <unistd.h>     // For close()
#include <sys/ioctl.h>  // For ioctl()

namespace motor_driver{
    class MotorDriver : public hardware_interface::ActuatorInterface{
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(MotorDriver)

            // Lifecycle Methods
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        private:
            int pwm_fd_ = -1;             // File descriptor for the PWM device
            int pwm_channel_ = 0;         // Which PWM channel to use (0 or 1)
            std::string device_path_;     // Path like "/dev/pwmchip0"
            
            double hw_command_ = 0.0;     // Velocity command from ROS
            double hw_state_ = 0.0;       // Feedback state


    };

}