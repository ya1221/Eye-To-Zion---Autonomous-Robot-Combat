#pragma once

// Ros2 includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// Image processing includes
#include <opencv2/opencv.hpp>

// Standard includes
#include <string>
#include <filesystem>

class MapToImage : public rclcpp::Node{
    public:
        MapToImage();
    private:
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
        void save_map(const cv::Mat& img);
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        bool folder_verified_ = false;
        inline static constexpr const char* TEMP_MAP_PATH = "/home/itay3711/AutonomousWarfare/src/robot_description/config/robot_mount/robot_images/temp.jpg";
        inline static  constexpr const char* REAL_MAP_PATH = "/home/itay3711/AutonomousWarfare/src/robot_description/config/robot_mount/robot_images/map.jpg";
        std::vector<int> compression_params_;
        inline static constexpr int JPEG_QUALITY = 90;
};
