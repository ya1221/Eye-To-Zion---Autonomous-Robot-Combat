#include "telemetry_data/telegraf_bridge.hpp"
#include <string>
#include <fmt/core.h>


using namespace std::chrono_literals;

TelegrafBridge::TelegrafBridge() : Node("telegraf_bridge"), telemetry_sender_()
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 
        10, 
        std::bind(&TelegrafBridge::pose_callback, this, std::placeholders::_1)
    );

    ammo_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/ammo", 
        10, 
        std::bind(&TelegrafBridge::ammo_callback, this, std::placeholders::_1)
    );

    health_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/health", 
        10, 
        std::bind(&TelegrafBridge::health_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Telegraf Bridge Initialized");
}

int64_t TelegrafBridge::get_timestamp(){
    return std::chrono::duration_cast<Nanoseconds>(Clock::now().time_since_epoch()).count();
}

void TelegrafBridge::pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{   
    std::string pose = fmt::format(
        "robot_pose,id=1 x={:.2f},y={:.2f} {}\n",
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y,
        get_timestamp()
    );

    telemetry_sender_.send_metric(pose);
}

void TelegrafBridge::path_callback(const nav_msgs::msg::Path::SharedPtr path_msg){
    std::string path_payload = "";
    for(const auto& pose_stamped : path_msg->poses){
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;

        path_payload += fmt::format(
            "robot_path,id=1 x={:.2f},y={:.2f} {}\n",
            x, y, get_timestamp()
        );
    }

    telemetry_sender_.send_metric(path_payload);
}

void TelegrafBridge::ammo_callback(const std_msgs::msg::Int32::SharedPtr ammo_msg)
{
    std::string ammo_payload = fmt::format(
        "robot_ammo,id=1 ammo={} {}\n",
        ammo_msg->data,
        get_timestamp()
    );

    telemetry_sender_.send_metric(ammo_payload);
}

void TelegrafBridge::health_callback(const std_msgs::msg::Int32::SharedPtr health_msg)
{
    std::string health_payload = fmt::format(
        "robot_health,id=1 health={} {}\n",
        health_msg->data,
        get_timestamp()
    );

    telemetry_sender_.send_metric(health_payload);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelegrafBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}