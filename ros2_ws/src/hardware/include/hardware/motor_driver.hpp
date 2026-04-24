#pragma once

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"


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

        private:
                


    };

}