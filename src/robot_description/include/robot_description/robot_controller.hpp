#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deque>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // <-- add this

class RobotController : public rclcpp::Node {
public:
  RobotController();

private:
  // --- Callbacks / Control ---
  void getPath(const nav_msgs::msg::Path::SharedPtr path);
  void getOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
  void stopRobot();
  void controlLoop();

  // --- Members ---
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<geometry_msgs::msg::PoseStamped> waypoints_;

  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool have_odom_{false};

  // --- Params ---
  double lookahead_dist_;
  double max_speed_;
  double k_heading_;
  double prune_dist_;
};
