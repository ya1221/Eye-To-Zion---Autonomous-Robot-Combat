#include "robot_description/map_to_image_node.hpp"
#include <filesystem>

MapToImage::MapToImage(): Node("map_to_image_node"){
    compression_params_ = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 
        rclcpp::QoS(1).transient_local(), 
        std::bind(&MapToImage::map_callback, this, std::placeholders::_1));
}

void MapToImage::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg){
    if(map_msg->data.empty()){
        RCLCPP_WARN(this->get_logger(), "Empty map received");
        return;
    }

    // 1. Wrap the raw data in a Mat (Zero copy)
    // We treat the data as unsigned char (0-255) to use a Lookup Table
    cv::Mat map_img(map_msg->info.height, map_msg->info.width, CV_8UC1, const_cast<signed char*>(map_msg->data.data()));

    // 2. Create the Lookup Table (Only done once usually, but fast enough to do here)
    cv::Mat lookUpTable(1, 256, CV_8U);
    lookUpTable = cv::Scalar(128); 
    uint8_t* p = lookUpTable.ptr();

    p[0] = 255;
    p[100] = 0;

    // 3. Apply the mapping (Instant conversion)
    cv::Mat color_map;
    cv::LUT(map_img, lookUpTable, color_map);

    cv::flip(color_map, color_map, 0);
    save_map(color_map);
}

void MapToImage::save_map(const cv::Mat& img){
    if (cv::imwrite(MapToImage::TEMP_MAP_PATH, img, compression_params_)) {
        if (std::rename(MapToImage::TEMP_MAP_PATH, MapToImage::REAL_MAP_PATH) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to rename temp file: %s", strerror(errno));
        }else{
            RCLCPP_INFO(this->get_logger(), "Map updated successfully.");
        }

    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not write temp map to %s", TEMP_MAP_PATH);
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapToImage>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}