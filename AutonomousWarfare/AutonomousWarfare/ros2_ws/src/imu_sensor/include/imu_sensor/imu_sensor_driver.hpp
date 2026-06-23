#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "imu_sensor/imu_sensor_registers.hpp"

#include <array>
#include <atomic>
#include <string>

namespace imu_sensor
{

class ImuDriverNode : public rclcpp::Node
{
public:
  explicit ImuDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImuDriverNode() override;

private:
  // ─── Initialization ──────────────────────────────────────────────────────
  void declare_parameters();
  bool open_i2c();
  bool init_icm20948();
  bool init_ak09916();
  void calibrate_gyro();
  void setup_publishers();
  void setup_timer();
  void setup_services();

  // ─── Main loop ───────────────────────────────────────────────────────────
  void read_and_publish();

  // ─── I2C low-level helpers ───────────────────────────────────────────────
  bool write_register(int fd, uint8_t reg, uint8_t value);
  bool read_register(int fd, uint8_t reg, uint8_t & value);
  bool read_registers(int fd, uint8_t start_reg, uint8_t * buffer, size_t length);
  bool select_bank(uint8_t bank);

  // ─── Sensor reading ──────────────────────────────────────────────────────
  bool read_accel_gyro(
    double & ax, double & ay, double & az,
    double & gx, double & gy, double & gz);
  bool read_magnetometer(double & mx, double & my, double & mz);
  bool read_temperature(double & temp_c);

  // ─── Service callbacks ───────────────────────────────────────────────────
  void calibrate_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ─── Helpers ─────────────────────────────────────────────────────────────
  double get_gyro_sensitivity() const;
  double get_accel_sensitivity() const;
  uint8_t get_gyro_fs_bits() const;
  uint8_t get_accel_fs_bits() const;

  // ─── I2C file descriptors ────────────────────────────────────────────────
  int icm_fd_{-1};   // File descriptor for ICM-20948
  int mag_fd_{-1};   // File descriptor for AK09916 (via I2C bypass)
  uint8_t current_bank_{0xFF};  // Track current register bank (0xFF = unknown)

  // ─── Parameters ──────────────────────────────────────────────────────────
  int i2c_bus_;
  int i2c_address_;
  std::string frame_id_;
  double publish_rate_;
  int gyro_range_dps_;
  int accel_range_g_;
  bool enable_magnetometer_;
  bool calibrate_on_startup_;
  int calibration_samples_;
  bool publish_temperature_;

  // ─── Gyro bias (from calibration) ────────────────────────────────────────
  double gyro_bias_x_{0.0};
  double gyro_bias_y_{0.0};
  double gyro_bias_z_{0.0};

  // ─── Publishers ──────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;

  // ─── Timer ───────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;

  // ─── Services ────────────────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;

  // ─── Covariance (diagonal, set from params) ──────────────────────────────
  double gyro_covariance_;
  double accel_covariance_;
  double mag_covariance_;
};

}  // namespace imu_sensor

