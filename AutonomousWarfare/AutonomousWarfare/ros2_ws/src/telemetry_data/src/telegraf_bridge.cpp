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

    std::random_device rd;
    rng_ = std::mt19937(rd());
    dist_ = std::uniform_int_distribution<int>(0, 100);

    
    timer_ = this->create_wall_timer(
        1000ms, 
        std::bind(&TelegrafBridge::health_callback, this)
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

void TelegrafBridge::health_callback(){
    int random_health = dist_(rng_);
    int random_fr_wheel_health = dist_(rng_);
    int random_fl_wheel_health = dist_(rng_);
    int random_rr_wheel_health = dist_(rng_);
    int random_rl_wheel_health = dist_(rng_);

    std::string health = fmt::format(
        "robot_health,id=1 avg_health={},fr_wheel={},fl_wheel={},rr_wheel={},rl_wheel={} {}\n",
        random_health,
        random_fr_wheel_health,
        random_fl_wheel_health,
        random_rr_wheel_health,
        random_rl_wheel_health,
        get_timestamp()
    );

    telemetry_sender_.send_metric(health);
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


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelegrafBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}