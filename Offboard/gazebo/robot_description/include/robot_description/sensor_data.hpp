#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <InfluxDB/InfluxDBFactory.h>
#include <InfluxDB/Point.h>

class SensorData: public rclcpp::Node{
    public:
        SensorData();

    private:
        void SendData(const nav_msgs::msg::Odometry::SharedPtr msg);

        std::unique_ptr<influxdb::InfluxDB> influx_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

};