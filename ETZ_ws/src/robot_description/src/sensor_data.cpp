#include "robot_description/sensor_data.hpp"
using std::placeholders::_1;

SensorData::SensorData() : rclcpp::Node("send_data"){
    // URL Format: http://:TOKEN@HOST:PORT?db=BUCKET&org=ORG
    // 1. The ':' tells it "No Username".
    // 2. The Token goes in the "Password" slot.
    // 3. Org and Bucket are query parameters.
    
    std::string connection_url = "http://:my_super_secret_robot_token_999@localhost:8086?db=sensor&org=eye_to_zion";

    influx_ = influxdb::InfluxDBFactory::Get(connection_url);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&SensorData::SendData, this, _1));
}

void SensorData::SendData(const nav_msgs::msg::Odometry::SharedPtr msg){
    auto point = influxdb::Point{"localization"}
            .addTag("robot_id", "robot_1")
            .addField("pos_x", msg->pose.pose.position.x)
            .addField("pos_y", msg->pose.pose.position.y);

    try {
        influx_->write(std::move(point));
    }
    catch (const std::exception &e) {
        // This prevents the node from crashing if the DB is down
        RCLCPP_ERROR(this->get_logger(), "InfluxDB Error: %s", e.what());
    }
}

int main(int argc, char * argv[])
{
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<SensorData>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
}