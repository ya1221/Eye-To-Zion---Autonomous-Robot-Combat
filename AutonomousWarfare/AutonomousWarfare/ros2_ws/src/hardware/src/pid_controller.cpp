#include "hardware/pid_controller.hpp"

HeadingPIDController::HeadingPIDController()
  : Node("heading_pid_controller"),
    rate_integral_(0.0),
    rate_prev_error_(0.0),
    heading_integral_(0.0),
    heading_prev_error_(0.0),
    actual_heading_(0.0),
    actual_yaw_rate_(0.0),
    cmd_linear_(0.0),
    cmd_angular_(0.0),
    held_heading_(0.0),
    odom_received_(false),
    imu_received_(false),
    cmd_vel_received_(false),
    held_heading_initialized_(false),
    last_cmd_time_(0, 0, RCL_ROS_TIME)
{
  // ---- Declare parameters ----

  // Yaw rate PID (inner loop - active during turns)
  this->declare_parameter("kp_rate", 0.5);
  this->declare_parameter("ki_rate", 0.0);
  this->declare_parameter("kd_rate", 0.05);

  // Heading hold PID (outer loop - active when driving straight)
  this->declare_parameter("kp_heading", 1.0);
  this->declare_parameter("ki_heading", 0.05);
  this->declare_parameter("kd_heading", 0.1);

  // Controller settings
  this->declare_parameter("loop_rate", 50.0);
  this->declare_parameter("integral_max", 0.5);
  this->declare_parameter("straight_threshold", 0.01);  // rad/s
  this->declare_parameter("max_correction", 1.0);       // rad/s clamp on PID output

  // ---- Read parameters ----
  kp_rate_ = this->get_parameter("kp_rate").as_double();
  ki_rate_ = this->get_parameter("ki_rate").as_double();
  kd_rate_ = this->get_parameter("kd_rate").as_double();

  kp_heading_ = this->get_parameter("kp_heading").as_double();
  ki_heading_ = this->get_parameter("ki_heading").as_double();
  kd_heading_ = this->get_parameter("kd_heading").as_double();

  loop_rate_ = this->get_parameter("loop_rate").as_double();
  integral_max_ = this->get_parameter("integral_max").as_double();
  straight_threshold_ = this->get_parameter("straight_threshold").as_double();
  max_correction_ = this->get_parameter("max_correction").as_double();

  // ---- Subscribers ----

  // EKF fused odometry (accurate heading, corrected by ArUco)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/global", 10,
    std::bind(&HeadingPIDController::odomCallback, this, std::placeholders::_1));

  // IMU direct (fast yaw rate)
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&HeadingPIDController::imuCallback, this, std::placeholders::_1));

  // Nav2 velocity commands (original, uncorrected)
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&HeadingPIDController::cmdVelCallback, this, std::placeholders::_1));

  // ---- Publisher ----
  // Corrected cmd_vel → ackermann_steering_controller listens to this
  cmd_vel_corrected_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_corrected", 10);

  // ---- Control loop timer ----
  double period_ms = 1000.0 / loop_rate_;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(period_ms)),
    std::bind(&HeadingPIDController::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Heading PID Controller (cmd_vel filter) started");
  RCLCPP_INFO(this->get_logger(),
    "  Rate PID:    kp=%.2f ki=%.2f kd=%.2f", kp_rate_, ki_rate_, kd_rate_);
  RCLCPP_INFO(this->get_logger(),
    "  Heading PID: kp=%.2f ki=%.2f kd=%.2f", kp_heading_, ki_heading_, kd_heading_);
  RCLCPP_INFO(this->get_logger(),
    "  Loop rate: %.0f Hz | Max correction: %.2f rad/s", loop_rate_, max_correction_);
}

// ===========================================================
//  Callbacks
// ===========================================================

void HeadingPIDController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  actual_heading_ = tf2::getYaw(q);
  odom_received_ = true;
}

void HeadingPIDController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  actual_yaw_rate_ = msg->angular_velocity.z;
  imu_received_ = true;
}

void HeadingPIDController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_linear_ = msg->linear.x;
  cmd_angular_ = msg->angular.z;
  cmd_vel_received_ = true;
  last_cmd_time_ = this->now();
}

// ===========================================================
//  Main control loop
// ===========================================================

void HeadingPIDController::controlLoop()
{
  // Wait until all sensors are online
  if (!odom_received_ || !imu_received_ || !cmd_vel_received_)
    return;

  // Safety: no cmd_vel for over 0.5s → pass through zero
  double cmd_age = (this->now() - last_cmd_time_).seconds();
  if (cmd_age > 0.5)
  {
    resetPID();
    auto stop_msg = geometry_msgs::msg::Twist();
    cmd_vel_corrected_pub_->publish(stop_msg);
    return;
  }

  double dt = 1.0 / loop_rate_;
  double correction = 0.0;

  bool driving_straight = std::abs(cmd_angular_) < straight_threshold_;

  if (driving_straight)
  {
    // =======================================================
    //  HEADING HOLD MODE (outer loop)
    //  Nav2 wants zero rotation → hold current heading
    //  Uses EKF heading (ArUco + IMU fused) — drift-free
    // =======================================================

    if (!held_heading_initialized_)
    {
      held_heading_ = actual_heading_;
      held_heading_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Heading hold locked at %.2f rad", held_heading_);
    }

    double error = normalizeAngle(held_heading_ - actual_heading_);

    double p = kp_heading_ * error;

    heading_integral_ += error * dt;
    heading_integral_ = std::clamp(heading_integral_, -integral_max_, integral_max_);
    double i = ki_heading_ * heading_integral_;

    double d = kd_heading_ * (error - heading_prev_error_) / dt;
    heading_prev_error_ = error;

    correction = p + i + d;

    // Reset rate PID state
    rate_integral_ = 0.0;
    rate_prev_error_ = 0.0;

    RCLCPP_DEBUG(this->get_logger(),
      "[HEADING HOLD] held=%.3f actual=%.3f error=%.3f correction=%.3f",
      held_heading_, actual_heading_, error, correction);
  }
  else
  {
    // =======================================================
    //  YAW RATE MODE (inner loop)
    //  Nav2 wants a turn → match the commanded rotation speed
    //  Uses IMU yaw rate directly — fast, no integration
    // =======================================================

    double error = cmd_angular_ - actual_yaw_rate_;

    double p = kp_rate_ * error;

    rate_integral_ += error * dt;
    rate_integral_ = std::clamp(rate_integral_, -integral_max_, integral_max_);
    double i = ki_rate_ * rate_integral_;

    double d = kd_rate_ * (error - rate_prev_error_) / dt;
    rate_prev_error_ = error;

    correction = p + i + d;

    // Release heading hold — will re-capture when straight again
    held_heading_initialized_ = false;
    heading_integral_ = 0.0;
    heading_prev_error_ = 0.0;

    RCLCPP_DEBUG(this->get_logger(),
      "[YAW RATE] cmd=%.3f actual=%.3f error=%.3f correction=%.3f",
      cmd_angular_, actual_yaw_rate_, error, correction);
  }

  // Clamp correction to prevent wild steering
  correction = std::clamp(correction, -max_correction_, max_correction_);

  // ---- Build corrected cmd_vel ----
  // linear.x passes through untouched
  // angular.z = original Nav2 command + PID correction
  auto corrected_msg = geometry_msgs::msg::Twist();
  corrected_msg.linear.x = cmd_linear_;
  corrected_msg.angular.z = cmd_angular_ + correction;

  cmd_vel_corrected_pub_->publish(corrected_msg);
}

// ===========================================================
//  Helpers
// ===========================================================

double HeadingPIDController::normalizeAngle(double angle)
{
  while (angle > M_PI)  angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void HeadingPIDController::resetPID()
{
  rate_integral_ = 0.0;
  rate_prev_error_ = 0.0;
  heading_integral_ = 0.0;
  heading_prev_error_ = 0.0;
  held_heading_initialized_ = false;
}

// ===========================================================
//  Main
// ===========================================================

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingPIDController>());
  rclcpp::shutdown();
  return 0;
}