#include "shooting/shooting_node.hpp"

#include <chrono>
#include <functional>

ShootingNode::ShootingNode() : Node("shooting_node") {
    pub_ = create_publisher<std_msgs::msg::Bool>("/shooting_cmd", 10);
    declare_parameter("fire_rate_hz", 2.0);   // shots per second — changeable at runtime via `ros2 param set`

    last_toggle_time_ = this->now();

    // Fast fixed-rate heartbeat (100 Hz). fire_rate_hz is re-read from the
    // parameter server on every tick, so runtime changes take effect on the
    // next tick — no restart, no timer recreation needed.
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&ShootingNode::tick, this));
}

void ShootingNode::set_firing(bool on) {
    firing_ = on;
}

void ShootingNode::tick() {
    if (!firing_) {
        if (state_) {
            state_ = false;
            publish_state();
        }
        return;
    }

    double hz = get_parameter("fire_rate_hz").as_double();
    if (hz <= 0.0) {
        return;
    }
    double half_period_sec = 1.0 / (2.0 * hz);

    rclcpp::Time now = this->now();
    if ((now - last_toggle_time_).seconds() >= half_period_sec) {
        state_ = !state_;
        last_toggle_time_ = now;
        publish_state();
    }
}

void ShootingNode::publish_state() {
    std_msgs::msg::Bool msg;
    msg.data = state_;
    pub_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShootingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}