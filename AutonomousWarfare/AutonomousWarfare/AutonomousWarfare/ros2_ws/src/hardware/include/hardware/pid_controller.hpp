#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

class HeadingPIDController : public rclcpp::Node
{
public:
  HeadingPIDController();

private:
  // ---- Callbacks ----
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // ---- Control ----
  void controlLoop();

  // ---- Helpers ----
  double normalizeAngle(double angle);
  void resetPID();

  // ---- Subscribers ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // ---- Publisher ----
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_corrected_pub_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- PID gains: yaw rate loop (active during turns) ----
  double kp_rate_;
  double ki_rate_;
  double kd_rate_;

  // ---- PID gains: heading hold loop (active when driving straight) ----
  double kp_heading_;
  double ki_heading_;
  double kd_heading_;

  // ---- PID state: yaw rate loop ----
  double rate_integral_;
  double rate_prev_error_;

  // ---- PID state: heading hold loop ----
  double heading_integral_;
  double heading_prev_error_;

  // ---- Sensor data ----
  double actual_heading_;       // from EKF (ArUco + IMU + odom fused)
  double actual_yaw_rate_;      // from IMU directly (fast)

  // ---- Command data (latest cmd_vel from Nav2) ----
  double cmd_linear_;
  double cmd_angular_;

  // ---- Heading hold ----
  double held_heading_;         // heading captured when robot goes straight

  // ---- Config ----
  double loop_rate_;
  double integral_max_;
  double straight_threshold_;   // below this cmd_angular, robot is "going straight"
  double max_correction_;       // clamp PID correction to prevent wild steering

  // ---- Flags ----
  bool odom_received_;
  bool imu_received_;
  bool cmd_vel_received_;
  bool held_heading_initialized_;

  // ---- Timing ----
  rclcpp::Time last_cmd_time_;
};

