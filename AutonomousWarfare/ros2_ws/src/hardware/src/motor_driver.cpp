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


bool MotorDriver::open_serial_port() {
    // Open the serial device in read-write, non-controlling-terminal, non-blocking mode
    this->serial_fd_ = open(this->serial_device_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (this->serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
            "Failed to open serial port '%s': %s",
            this->serial_device_path_.c_str(), std::strerror(errno));
        return false;
    }

    // Clear the non-blocking flag so writes block until complete
    fcntl(this->serial_fd_, F_SETFL, 0);

    // Configure the serial port with termios
    struct termios tty;
    if (tcgetattr(this->serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
            "tcgetattr failed for '%s': %s",
            this->serial_device_path_.c_str(), std::strerror(errno));
        close(this->serial_fd_);
        this->serial_fd_ = -1;
        return false;
    }

    // Set baud rate (115200)
    speed_t baud;
    switch (this->serial_baud_) {
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        default:     baud = B115200; break;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1 (8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;    // No parity
    tty.c_cflag &= ~CSTOPB;    // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8 data bits

    // No hardware flow control
    tty.c_cflag &= ~CRTSCTS;

    // Enable receiver, local mode (ignore modem control lines)
    tty.c_cflag |= (CLOCAL | CREAD);

    // Raw input mode (no canonical processing, no echo, no signals)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // No software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Raw output mode
    tty.c_oflag &= ~OPOST;

    // Read timeout configuration: non-blocking with no minimum bytes
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    // Apply the configuration
    if (tcsetattr(this->serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
            "tcsetattr failed for '%s': %s",
            this->serial_device_path_.c_str(), std::strerror(errno));
        close(this->serial_fd_);
        this->serial_fd_ = -1;
        return false;
    }

    // Flush any stale data in the buffers
    tcflush(this->serial_fd_, TCIOFLUSH);

    return true;
}


hardware_interface::CallbackReturn MotorDriver::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->device_path_ = this->info_.hardware_parameters.at("pwm_device");

    // Read GPIO chip name for libgpiod (e.g. "gpiochip4" on Pi 5)
    if (this->info_.hardware_parameters.find("gpio_chip") != this->info_.hardware_parameters.end()) {
        this->gpio_chip_name_ = this->info_.hardware_parameters.at("gpio_chip");
    }

    // Read serial port configuration for Arduino steering control
    if (this->info_.hardware_parameters.find("serial_device") != this->info_.hardware_parameters.end()) {
        this->serial_device_path_ = this->info_.hardware_parameters.at("serial_device");
    }
    if (this->info_.hardware_parameters.find("serial_baud") != this->info_.hardware_parameters.end()) {
        this->serial_baud_ = std::stoi(this->info_.hardware_parameters.at("serial_baud"));
    }

    // Read optional PWM multiplier (default = 2.0)
    if (this->info_.hardware_parameters.find("pwm_multiplier") != this->info_.hardware_parameters.end()) {
        this->pwm_multiplier_ = std::stod(this->info_.hardware_parameters.at("pwm_multiplier"));
    }
    RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
        "PWM multiplier loaded: %.4f (override via hardware_parameter 'pwm_multiplier')",
        this->pwm_multiplier_);

    size_t total_states = 0;
    size_t total_commands = 0;

    // Dynamically map memory based on whatever is inside the URDF
    for (const auto & joint : info_.joints) {
        
        StateUpdater updater;

        for (size_t i = 0; i < joint.command_interfaces.size(); ++i) {
            if (joint.command_interfaces[i].name == "velocity") updater.cmd_vel_idx = total_commands + i;
            if (joint.command_interfaces[i].name == "position") updater.cmd_pos_idx = total_commands + i;
        }

        for (size_t i = 0; i < joint.state_interfaces.size(); ++i) {
            if (joint.state_interfaces[i].name == "velocity") updater.state_vel_idx = total_states + i;
            if (joint.state_interfaces[i].name == "position") updater.state_pos_idx = total_states + i;
        }

        if (updater.state_pos_idx != -1 || updater.state_vel_idx != -1) {
            this->state_updaters_.push_back(updater);
        }

        // 1. Check if this joint has a PWM channel assigned in the URDF
        int pwm_channel = -1;
        if (joint.parameters.find("pwm_channel") != joint.parameters.end()) {
            pwm_channel = std::stoi(joint.parameters.at("pwm_channel"));
        }

        // 2. If it has a channel AND accepts commands, add it to the active motors list
        if (pwm_channel != -1 && !joint.command_interfaces.empty()) {
            MotorTarget target;
            target.pwm_channel = pwm_channel;
            
            // For PWM output, we use the velocity command. If not found, fallback to the first command.
            target.cmd_index = (updater.cmd_vel_idx != -1) ? updater.cmd_vel_idx : total_commands; 

            // Read L298N direction GPIO pins from URDF (BCM line offsets)
            if (joint.parameters.find("in1_gpio") != joint.parameters.end()) {
                target.in1_gpio = std::stoi(joint.parameters.at("in1_gpio"));
            }
            if (joint.parameters.find("in2_gpio") != joint.parameters.end()) {
                target.in2_gpio = std::stoi(joint.parameters.at("in2_gpio"));
            }
            this->active_motors_.push_back(target);
        }

        // 3. Check if this joint is a steering servo (controlled via Arduino serial)
        if (joint.parameters.find("servo_index") != joint.parameters.end()) {
            ServoTarget servo;
            servo.servo_index = std::stoi(joint.parameters.at("servo_index"));
            // Steering uses position command
            servo.cmd_index = (updater.cmd_pos_idx != -1) ? updater.cmd_pos_idx : total_commands;
            this->active_servos_.push_back(servo);

            RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
                "Registered steering servo: joint='%s', servo_index=%d, cmd_index=%zu",
                joint.name.c_str(), servo.servo_index, servo.cmd_index);
        }

        // 4. Keep counting to allocate enough memory for everything
        total_states += joint.state_interfaces.size();
        total_commands += joint.command_interfaces.size();
    }

    // Sort servos by index so we always send left (0) before right (1)
    std::sort(this->active_servos_.begin(), this->active_servos_.end(),
        [](const ServoTarget& a, const ServoTarget& b) {
            return a.servo_index < b.servo_index;
        });

    // Allocate memory ONCE. The pointers will remain stable forever.
    this->hw_states_.resize(total_states, 0.0);
    this->hw_commands_.resize(total_commands, 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorDriver::on_activate(const rclcpp_lifecycle::State &) {

    // --- Open libgpiod chip (shared by all motors) ---
    if (!this->gpio_chip_name_.empty()) {
        this->gpio_chip_ = gpiod_chip_open_by_name(this->gpio_chip_name_.c_str());
        if (!this->gpio_chip_) {
            RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
                "Failed to open GPIO chip '%s': %s",
                this->gpio_chip_name_.c_str(), std::strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
            "Opened GPIO chip '%s' for L298N direction control",
            this->gpio_chip_name_.c_str());
    }

    for (auto & motor : this->active_motors_) {
        // --- PWM setup (unchanged) ---

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

        // --- L298N direction GPIO setup via libgpiod ---
        if (motor.in1_gpio >= 0) {
            motor.in1_line = gpiod_chip_get_line(this->gpio_chip_, motor.in1_gpio);
            if (motor.in1_line) {
                if (gpiod_line_request_output(motor.in1_line, "motor_driver", 0) < 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
                        "Failed to request GPIO line %d as output for IN1: %s",
                        motor.in1_gpio, std::strerror(errno));
                    motor.in1_line = nullptr;
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
                    "Failed to get GPIO line %d for IN1: %s",
                    motor.in1_gpio, std::strerror(errno));
            }
        }

        if (motor.in2_gpio >= 0) {
            motor.in2_line = gpiod_chip_get_line(this->gpio_chip_, motor.in2_gpio);
            if (motor.in2_line) {
                if (gpiod_line_request_output(motor.in2_line, "motor_driver", 0) < 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
                        "Failed to request GPIO line %d as output for IN2: %s",
                        motor.in2_gpio, std::strerror(errno));
                    motor.in2_line = nullptr;
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
                    "Failed to get GPIO line %d for IN2: %s",
                    motor.in2_gpio, std::strerror(errno));
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
            "Motor on PWM ch%d activated: IN1(gpio%d)=%s, IN2(gpio%d)=%s",
            motor.pwm_channel,
            motor.in1_gpio, motor.in1_line ? "OK" : "N/A",
            motor.in2_gpio, motor.in2_line ? "OK" : "N/A");
    }

    // --- Open serial port for Arduino steering servo control ---
    if (!this->serial_device_path_.empty() && !this->active_servos_.empty()) {
        if (!open_serial_port()) {
            RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
                "Failed to open serial port for steering servos. Steering will be unavailable.");
            // Non-fatal: drive motors can still work without steering
        } else {
            // Wait for Arduino bootloader to finish its reset cycle after USB serial open.
            // The Arduino Uno resets when DTR is asserted on serial port open.
            RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
                "Serial port '%s' opened at %d baud. Waiting 2s for Arduino bootloader...",
                this->serial_device_path_.c_str(), this->serial_baud_);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
                "Arduino ready. Steering servo control active (%zu servos).",
                this->active_servos_.size());
        }
    }

    // --- Internal node for shooting topic subscription ---
    // Hardware interfaces don't have their own node handle, so we create a lightweight
    // internal node. Its callbacks are processed via spin_some() inside read().
    this->internal_node_ = rclcpp::Node::make_shared("motor_driver_internal");
    this->flag_sub_ = this->internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/shooting_cmd", rclcpp::QoS(1),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            this->flag_active_.store(msg->data);
            RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
                "Shooting command received: %s", msg->data ? "FIRE" : "CEASE");
        });
    RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
        "Subscribed to /shooting_cmd for flag pin control.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorDriver::on_deactivate(const rclcpp_lifecycle::State &) {
    for (auto & motor : this->active_motors_) {
        // Release direction GPIO lines
        if (motor.in1_line) {
            gpiod_line_set_value(motor.in1_line, 0);
            gpiod_line_release(motor.in1_line);
            motor.in1_line = nullptr;
        }
        if (motor.in2_line) {
            gpiod_line_set_value(motor.in2_line, 0);
            gpiod_line_release(motor.in2_line);
            motor.in2_line = nullptr;
        }

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

    // Close the GPIO chip
    if (this->gpio_chip_) {
        gpiod_chip_close(this->gpio_chip_);
        this->gpio_chip_ = nullptr;
    }

    // --- Close serial port for steering ---
    if (this->serial_fd_ >= 0) {
        // Safety: deactivate shooting flag before closing
        const char* flag_off = "F0\n";
        ::write(this->serial_fd_, flag_off, strlen(flag_off));

        // Send a zeroed steering command before closing so servos center
        const char* zero_cmd = "S0.0000,0.0000\n";
        ::write(this->serial_fd_, zero_cmd, strlen(zero_cmd));

        // Small delay to let the data flush out before closing
        tcdrain(this->serial_fd_);
        close(this->serial_fd_);
        this->serial_fd_ = -1;
        RCLCPP_INFO(rclcpp::get_logger("motor_driver"), "Serial port closed. Steering servos centered. Shooting deactivated.");
    }

    // --- Tear down internal node ---
    this->flag_sub_.reset();
    this->internal_node_.reset();
    this->flag_active_.store(false);
    this->flag_last_sent_ = false;

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

hardware_interface::return_type MotorDriver::read(const rclcpp::Time &, const rclcpp::Duration & period) {
    // Process any pending shooting topic callbacks (non-blocking)
    if (this->internal_node_) {
        rclcpp::spin_some(this->internal_node_);
    }

    // Open-Loop mapping: update states based on commands for ALL joints
    double dt = period.seconds();

    for (const auto& updater : this->state_updaters_) {
        // If we have a position command, directly map it to position state
        if (updater.cmd_pos_idx != -1 && updater.state_pos_idx != -1) {
            this->hw_states_[updater.state_pos_idx] = this->hw_commands_[updater.cmd_pos_idx];
        }

        // If we have a velocity command, map it to velocity state, and integrate into position state
        if (updater.cmd_vel_idx != -1) {
            double vel = this->hw_commands_[updater.cmd_vel_idx];
            
            if (updater.state_vel_idx != -1) {
                this->hw_states_[updater.state_vel_idx] = vel;
            }
            
            // Only integrate if there was no position command explicitly overriding it
            if (updater.state_pos_idx != -1 && updater.cmd_pos_idx == -1) {
                this->hw_states_[updater.state_pos_idx] += vel * dt;
            }
        }
    }
    
    return hardware_interface::return_type::OK; 
}

hardware_interface::return_type MotorDriver::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // === Drive Motors (PWM + L298N direction via GPIO) ===
    for (const auto& motor : this->active_motors_) {
        if (motor.duty_cycle_fd < 0) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 2000,
                "PWM channel %d: fd is invalid (%d)", motor.pwm_channel, motor.duty_cycle_fd);
            continue;
        }

        // Calculate the duty cycle in nanoseconds
        double target_cmd = this->hw_commands_[motor.cmd_index];
        uint64_t duty_cycle_ns = 0;
        if(motor.pwm_channel == 0){
            duty_cycle_ns = static_cast<uint64_t>(this->pwm_multiplier_ * std::abs(target_cmd) * 2000000.0);
        } else {
            duty_cycle_ns = static_cast<uint64_t>(std::abs(target_cmd) * 2000000.0);
        }

        // Clamp to period (20ms = 20,000,000 ns)
        if (duty_cycle_ns > 20000000) {
            duty_cycle_ns = 20000000;
        }

        // L298N direction control via libgpiod: set IN1/IN2 based on command sign
        if (motor.in1_line && motor.in2_line) {
            if (target_cmd > 0.0) {
                gpiod_line_set_value(motor.in1_line, 1);  // Forward
                gpiod_line_set_value(motor.in2_line, 0);
            } else if (target_cmd < 0.0) {
                gpiod_line_set_value(motor.in1_line, 0);  // Backward
                gpiod_line_set_value(motor.in2_line, 1);
            } else {
                gpiod_line_set_value(motor.in1_line, 0);  // Coast
                gpiod_line_set_value(motor.in2_line, 0);
            }
        }

        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 1000,
            "PWM ch%d: cmd_index=%zu, target_cmd=%.4f, duty_cycle_ns=%lu, fd=%d",
            motor.pwm_channel, motor.cmd_index, target_cmd, duty_cycle_ns, motor.duty_cycle_fd);

        // REAL-TIME SAFE CONVERSION:
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "%lu", duty_cycle_ns);

        // REAL-TIME SAFE WRITE:
        ssize_t ret = pwrite(motor.duty_cycle_fd, buf, len, 0);
        if (ret < 0) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 2000,
                "PWM ch%d: pwrite failed! errno=%d (%s), buf='%s'",
                motor.pwm_channel, errno, std::strerror(errno), buf);
        }
    }

    // === Steering Servos (via Arduino serial UART) ===
    if (this->serial_fd_ >= 0 && !this->active_servos_.empty()) {
        // Build the command string: S<left_rad>,<right_rad>\n
        // active_servos_ is sorted by servo_index (0=left, 1=right)
        char cmd_buf[64];
        int cmd_len;

        if (this->active_servos_.size() >= 2) {
            double left_rad  = this->hw_commands_[this->active_servos_[0].cmd_index];
            double right_rad = this->hw_commands_[this->active_servos_[1].cmd_index];

            cmd_len = snprintf(cmd_buf, sizeof(cmd_buf), "S%.4f,%.4f\n", left_rad, right_rad);
        } else {
            // Single servo fallback
            double rad = this->hw_commands_[this->active_servos_[0].cmd_index];
            cmd_len = snprintf(cmd_buf, sizeof(cmd_buf), "S%.4f\n", rad);
        }

        ssize_t ret = ::write(this->serial_fd_, cmd_buf, cmd_len);
        if (ret < 0) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 2000,
                "Serial write failed: errno=%d (%s)", errno, std::strerror(errno));
        }

        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("motor_driver"), this->steady_clock_, 1000,
            "Steering serial: %.*s", cmd_len - 1, cmd_buf);  // -1 to strip the \n for logging
    }

    // === Shooting Flag (via Arduino serial UART — send only on state change) ===
    if (this->serial_fd_ >= 0) {
        bool current = this->flag_active_.load();
        if (current != this->flag_last_sent_) {
            const char* cmd = current ? "F1\n" : "F0\n";
            ::write(this->serial_fd_, cmd, strlen(cmd));
            this->flag_last_sent_ = current;

            RCLCPP_INFO(rclcpp::get_logger("motor_driver"),
                "Shooting flag sent to Arduino: %s", current ? "F1 (ACTIVE)" : "F0 (OFF)");
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