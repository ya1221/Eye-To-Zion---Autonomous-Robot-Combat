#include "robot_description/robot_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <functional>
#include <deque>
#include <cmath>

using std::placeholders::_1;

static double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}
static double dist2(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1, dy = y2 - y1;
  return dx*dx + dy*dy;
}

RobotController::RobotController() : rclcpp::Node("robot_controller"),
  lookahead_dist_(declare_parameter("lookahead_dist", 0.8)),
  max_speed_(declare_parameter("max_speed", 0)),
  k_heading_(declare_parameter("k_heading", 1.5)),
  prune_dist_(declare_parameter("prune_dist", 0.35)),
  have_odom_(false)
{
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10, std::bind(&RobotController::getPath, this, _1));

  // Publish generated Twist commands to the controller input topic (cmd_vel)
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", 10);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ackermann_steering_controller/odometry", 20, std::bind(&RobotController::getOdom, this, _1));

  timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RobotController::controlLoop, this));  // 10 Hz control
}

void RobotController::getPath(const nav_msgs::msg::Path::SharedPtr path) {
  if (path->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "The positions in path are empty");
    return;
  }
  waypoints_.clear();
  for (const auto &ps : path->poses) {
    waypoints_.push_back(ps);
  }
}

void RobotController::getOdom(const nav_msgs::msg::Odometry::SharedPtr odom) {
  x_ = odom->pose.pose.position.x;
  y_ = odom->pose.pose.position.y;
  yaw_ = tf2::getYaw(odom->pose.pose.orientation);

  have_odom_ = std::isfinite(x_) && std::isfinite(y_) && std::isfinite(yaw_)
               && (odom->child_frame_id == "base_link");

  if (!have_odom_) {
    RCLCPP_WARN(get_logger(),
      "Invalid odom (x=%.3f y=%.3f yaw=%.3f child='%s')",
      x_, y_, yaw_, odom->child_frame_id.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Odom: x=%.3f y=%.3f yaw=%.3f", x_, y_, yaw_);
  }
}


void RobotController::stopRobot() {
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

void RobotController::controlLoop() {
  //  if (!have_odom_) {
  //   // Let the controller start odom by sending a small, safe command
  //   geometry_msgs::msg::TwistStamped cmd;
  //   cmd.header.stamp = now();
  //   cmd.header.frame_id = "base_link";
  //   cmd.twist.linear.x = 0.1;   // tiny forward nudge
  //   cmd.twist.angular.z = 0.0;
  //   cmd_vel_pub_->publish(cmd);
  //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odom… (nudging)");
  //   return;
  // }

  // while (!waypoints_.empty() &&
  //        dist2(x_, y_, waypoints_.front().pose.position.x, waypoints_.front().pose.position.y)
  //          < prune_dist_ * prune_dist_) {
  //   waypoints_.pop_front();
  // }

  // if (waypoints_.empty()) {
  //   stopRobot();
  //   return;
  // }

  // auto target = waypoints_.front();
  // for (const auto &wp : waypoints_) {
  //   if (std::sqrt(dist2(x_, y_, wp.pose.position.x, wp.pose.position.y)) >= lookahead_dist_) {
  //     target = wp; break;
  //   }
  // }

  // const double dx = target.pose.position.x - x_;
  // const double dy = target.pose.position.y - y_;
  // const double target_angle = std::atan2(dy, dx);
  // const double heading_err = normalizeAngle(target_angle - yaw_);

  // geometry_msgs::msg::TwistStamped cmd;
  // cmd.header.stamp = now();
  // cmd.header.frame_id = "base_link";
  // cmd.twist.linear.x  = max_speed_;
  // cmd.twist.angular.z = k_heading_ * heading_err;
  // cmd_vel_pub_->publish(cmd);
  // RCLCPP_INFO(get_logger(), "cmd vx=%.3f wz=%.3f", cmd.twist.linear.x, cmd.twist.angular.z);

  // geometry_msgs::msg::TwistStamped cmd;
  // cmd.header.stamp = now();
  // cmd.header.frame_id = "base_link";
  // cmd.twist.linear.x  = max_speed_;
  // cmd.twist.angular.z = 0;
  // cmd_vel_pub_->publish(cmd);
  // RCLCPP_INFO(get_logger(), "cmd vx=%.3f wz=%.3f", cmd.twist.linear.x, cmd.twist.angular.z);
}

int main(int argc, char** argv) {
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<RobotController>());
  // rclcpp::shutdown();
  // return 0;
}
