#include "telemetry_data/telegraf_bridge.hpp"
#include <string>
#include <fmt/core.h>


using namespace std::chrono_literals;

TelegrafBridge::TelegrafBridge() : Node("telegraf_bridge"), telemetry_sender_()
{
    this->declare_parameter<int>("num_teams", 0);    
    int num_teams = this->get_parameter("num_teams").as_int();
    teams_sub_.reserve(num_teams);

   for (int i = 0; i < num_teams; i++){
        std::string topic_name = "teams_team/team_" + std::to_string(i) + "/zone_occupied";

        auto sub = this->create_subscription<std_msgs::msg::Bool>(
            topic_name, 10,
            [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
                team_occupied_callback(msg, i);
            });

        teams_sub_.push_back(sub);
    }

    RCLCPP_INFO(this->get_logger(), "Telegraf Bridge Initialized");
}

int64_t TelegrafBridge::get_timestamp(){
    return std::chrono::duration_cast<Nanoseconds>(Clock::now().time_since_epoch()).count();
}



void TelegrafBridge::team_occupied_callback(
    const std_msgs::msg::Bool::SharedPtr msg, int team_id)
{
    std::string occupied = fmt::format(
        "robot_occupied,id={} occupied={} {}\n",
        team_id,
        msg->data ? "true" : "false",
        get_timestamp()
    );

    telemetry_sender_.send_metric(occupied);
    RCLCPP_INFO(this->get_logger(), "[Telegraf Bridge] Sent metric: %s", occupied.c_str());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelegrafBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}