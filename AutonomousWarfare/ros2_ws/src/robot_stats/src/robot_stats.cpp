#include "robot_stats/robot_stats.hpp"

static constexpr int DEFAULT_AMMO = 100;
static constexpr int DEFAULT_HEALTH = 100;

RobotStats::RobotStats() : Node("robot_stats")
{
    // Declare the ammo parameter (configurable from launch param file)
    this->declare_parameter("ammo", DEFAULT_AMMO);
    this->declare_parameter("health", DEFAULT_HEALTH);
    ammo_ = this->get_parameter("ammo").as_int();
    health_ = this->get_parameter("health").as_int();

    // --- Subscribers ---
    impact_alert_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/audio/impact_alert", 10,
        std::bind(&RobotStats::impact_alert_callback, this, std::placeholders::_1)
    );

    shooting_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/shooting_cmd", 10,
        std::bind(&RobotStats::shooting_cmd_callback, this, std::placeholders::_1)
    );

    // --- Publishers ---
    ammo_pub_ = this->create_publisher<std_msgs::msg::Int32>("/ammo", 10);

    health_pub_ = this->create_publisher<std_msgs::msg::Int32>("/health", 10);

    RCLCPP_INFO(this->get_logger(), "RobotStats node initialized  —  ammo=%d", ammo_);
}

void RobotStats::impact_alert_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Impact alert received: %s", msg->data.c_str());
    if (!msg->data.empty()) {
        if (health_ > 0) {
            health_--;
            RCLCPP_INFO(this->get_logger(), "Hit! Health remaining: %d", health_);
        } else {
            RCLCPP_WARN(this->get_logger(), "No health left!");
        }

        // Publish updated health count
        std_msgs::msg::Int32 health_msg;
        health_msg.data = health_;
        health_pub_->publish(health_msg);
    }
}

void RobotStats::shooting_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        if (ammo_ > 0) {
            ammo_--;
            RCLCPP_INFO(this->get_logger(), "Shot fired! Ammo remaining: %d", ammo_);
        } else {
            RCLCPP_WARN(this->get_logger(), "No ammo left!");
        }

        // Publish updated ammo count
        std_msgs::msg::Int32 ammo_msg;
        ammo_msg.data = ammo_;
        ammo_pub_->publish(ammo_msg);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStats>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
