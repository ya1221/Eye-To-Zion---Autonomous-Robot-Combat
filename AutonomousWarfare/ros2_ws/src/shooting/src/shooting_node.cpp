#include "shooting/shooting_node.hpp"

#include <chrono>
#include <functional>

static constexpr double DEFAULT_FIRE_RATE_HZ = 2.0;

ShootingNode::ShootingNode() : Node("shooting_node") {
    pub_ = create_publisher<std_msgs::msg::Bool>("/shooting_cmd", 10);

    declare_parameter("fire_rate_hz", DEFAULT_FIRE_RATE_HZ);

    param_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&ShootingNode::on_parameters_changed, this, std::placeholders::_1));

    mode_sub_ = create_subscription<std_msgs::msg::String>(
        "/shooting_mode", 10,
        std::bind(&ShootingNode::mode_callback, this, std::placeholders::_1));

    firing_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/firing", 10,
        std::bind(&ShootingNode::firing_callback, this, std::placeholders::_1));

    fire_once_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/fire_once",
        std::bind(&ShootingNode::fire_once_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    last_shot_time_ = this->now();

    // 10ms fast heartbeat for precise auto rate toggling
    auto_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                    std::bind(&ShootingNode::auto_tick, this));
    
    // 250ms slow timer for single mode pulse
    single_timer_ = create_wall_timer(std::chrono::milliseconds(250),
                                      std::bind(&ShootingNode::single_tick, this));

    RCLCPP_INFO(get_logger(),
                "ShootingNode started  —  mode=%s  rate=%.1f Hz",
                fire_mode_.c_str(), get_parameter("fire_rate_hz").as_double());
}

void ShootingNode::mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& new_mode = msg->data;
    if (new_mode != "auto" && new_mode != "single") {
        RCLCPP_WARN(get_logger(), "Unknown mode '%s'", new_mode.c_str());
        return;
    }
    if (new_mode == fire_mode_) return;

    fire_mode_ = new_mode;
    if (fire_mode_ == "single") {
        internal_rate_reset_ = true;
        set_parameter(rclcpp::Parameter("fire_rate_hz", DEFAULT_FIRE_RATE_HZ));
        internal_rate_reset_ = false;
        single_shot_pending_ = false;
        firing_ = false;
        
        // Ensure we stop any auto shooting when switching to single mode
        if (current_cmd_state_) {
            current_cmd_state_ = false;
            publish_cmd(false);
        }
    } else {
        last_shot_time_ = this->now();
    }
    RCLCPP_INFO(get_logger(), "Mode changed to %s", fire_mode_.c_str());
}

void ShootingNode::firing_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    firing_ = msg->data;
    RCLCPP_INFO(get_logger(), "Firing %s", firing_ ? "ENABLED" : "DISABLED");
    if (!firing_) {
        if (fire_mode_ == "auto" && current_cmd_state_) {
            current_cmd_state_ = false;
            publish_cmd(false);
        }
    }
}

rcl_interfaces::msg::SetParametersResult ShootingNode::on_parameters_changed(
        const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& p : params) {
        if (p.get_name() == "fire_rate_hz") {
            if (fire_mode_ == "single" && !internal_rate_reset_) {
                result.successful = false;
                result.reason = "fire_rate_hz locked in single mode";
                return result;
            }
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() <= 0.0) {
                result.successful = false;
                return result;
            }
        }
    }
    return result;
}

void ShootingNode::fire_once_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (fire_mode_ != "single") {
        response->success = false;
        response->message = "Mode is not single";
        return;
    }

    single_shot_pending_ = true;
    response->success = true;
    response->message = "Single shot queued";
    RCLCPP_INFO(get_logger(), "Single shot queued");
}

void ShootingNode::single_tick() {
    if (fire_mode_ != "single") {
        return; 
    }

    if (single_shot_pending_) {
        if (!current_cmd_state_) {
            // Rising edge: Start the shot
            current_cmd_state_ = true;
            publish_cmd(true);
        } else {
            // Falling edge: This happens on the NEXT tick (250ms later)
            current_cmd_state_ = false;
            publish_cmd(false);
            single_shot_pending_ = false;
        }
    } else if (current_cmd_state_) {
        // Failsafe catch
        current_cmd_state_ = false;
        publish_cmd(false);
    }
}

void ShootingNode::auto_tick() {
    if (!firing_ || fire_mode_ != "auto") {
        return; 
    }

    double hz = get_parameter("fire_rate_hz").as_double();
    if (hz <= 0.0) return;
    
    // Toggle state in auto mode to shoot at the given rate
    double half_period_sec = 1.0 / (2.0 * hz);
    rclcpp::Time now = this->now();
    if ((now - last_shot_time_).seconds() >= half_period_sec) {
        current_cmd_state_ = !current_cmd_state_;
        last_shot_time_ = now;
        publish_cmd(current_cmd_state_);
    }
}

void ShootingNode::publish_cmd(bool value) {
    std_msgs::msg::Bool msg;
    msg.data = value;
    RCLCPP_INFO(get_logger(), "Shooting command published: %s", value ? "True" : "False");
    pub_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShootingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}