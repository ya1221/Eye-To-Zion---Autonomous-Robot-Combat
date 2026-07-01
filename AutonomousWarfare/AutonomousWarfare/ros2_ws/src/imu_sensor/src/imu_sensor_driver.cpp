#include "imu_sensor/imu_sensor_driver.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
extern "C" {
  #include <i2c/smbus.h>
}
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace imu_sensor
{

// ═══════════════════════════════════════════════════════════════════════════════
// Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════════════════

ImuDriverNode::ImuDriverNode(const rclcpp::NodeOptions & options)
: Node("icm20948_driver", options)
{
  declare_parameters();

  // Load parameters
  i2c_bus_              = this->get_parameter("i2c_bus").as_int();
  i2c_address_          = this->get_parameter("i2c_address").as_int();
  frame_id_             = this->get_parameter("frame_id").as_string();
  publish_rate_         = this->get_parameter("publish_rate").as_double();
  gyro_range_dps_       = this->get_parameter("gyro_range_dps").as_int();
  accel_range_g_        = this->get_parameter("accel_range_g").as_int();
  enable_magnetometer_  = this->get_parameter("enable_magnetometer").as_bool();
  calibrate_on_startup_ = this->get_parameter("calibrate_on_startup").as_bool();
  calibration_samples_  = this->get_parameter("calibration_samples").as_int();
  publish_temperature_  = this->get_parameter("publish_temperature").as_bool();
  gyro_covariance_      = this->get_parameter("gyro_covariance").as_double();
  accel_covariance_     = this->get_parameter("accel_covariance").as_double();
  mag_covariance_       = this->get_parameter("mag_covariance").as_double();

  RCLCPP_INFO(this->get_logger(),
    "Starting ICM-20948 driver: bus=%d, addr=0x%02X, rate=%.0f Hz, "
    "gyro=%d dps, accel=%d g, mag=%s",
    i2c_bus_, i2c_address_, publish_rate_, gyro_range_dps_, accel_range_g_,
    enable_magnetometer_ ? "enabled" : "disabled");

  // Initialize hardware
  if (!open_i2c()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open I2C bus. Node will not start.");
    return;
  }
  if (!init_icm20948()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize ICM-20948. Node will not start.");
    return;
  }
  if (enable_magnetometer_ && !init_ak09916()) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to initialize AK09916 magnetometer. Continuing without mag.");
    enable_magnetometer_ = false;
    select_bank(0);
    write_register(bank0::USER_CTRL, 0x00); 
    write_register(bank0::INT_PIN_CFG, 0x00);
  }

  // Calibrate gyro
  if (calibrate_on_startup_) {
    RCLCPP_INFO(this->get_logger(),
      "Calibrating gyro bias (%d samples). Keep the robot STILL...", calibration_samples_);
    calibrate_gyro();
    RCLCPP_INFO(this->get_logger(),
      "Gyro bias: x=%.6f, y=%.6f, z=%.6f rad/s",
      gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
  }

  setup_publishers();
  setup_services();
  setup_timer();

  RCLCPP_INFO(this->get_logger(), "ICM-20948 driver initialized and publishing.");
}

ImuDriverNode::~ImuDriverNode()
{
  if (i2c_fd_ >= 0) close(i2c_fd_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Parameter Declaration
// ═══════════════════════════════════════════════════════════════════════════════

void ImuDriverNode::declare_parameters()
{
  this->declare_parameter<int>("i2c_bus", 1);
  this->declare_parameter<int>("i2c_address", ICM20948_ADDR_LOW);
  this->declare_parameter<std::string>("frame_id", "imu_link");
  this->declare_parameter<double>("publish_rate", 100.0);
  this->declare_parameter<int>("gyro_range_dps", 500);
  this->declare_parameter<int>("accel_range_g", 4);
  this->declare_parameter<bool>("enable_magnetometer", true);
  this->declare_parameter<bool>("calibrate_on_startup", true);
  this->declare_parameter<int>("calibration_samples", 200);
  this->declare_parameter<bool>("publish_temperature", false);

  // Default covariances — tune these based on your sensor's actual noise
  this->declare_parameter<double>("gyro_covariance", 0.0004);   // (rad/s)²
  this->declare_parameter<double>("accel_covariance", 0.01);    // (m/s²)²
  this->declare_parameter<double>("mag_covariance", 0.01);      // Tesla²
}

// ═══════════════════════════════════════════════════════════════════════════════
// I2C Initialization
// ═══════════════════════════════════════════════════════════════════════════════

bool ImuDriverNode::open_i2c()
{
  std::string i2c_path = "/dev/i2c-" + std::to_string(i2c_bus_);

  // Open file descriptor for ICM-20948
  i2c_fd_ = open(i2c_path.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open I2C bus at %s: %s",
      i2c_path.c_str(), strerror(errno));
    return false;
  }
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Cannot set I2C address 0x%02X: %s",
      i2c_address_, strerror(errno));
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "I2C bus %s opened successfully.", i2c_path.c_str());
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ICM-20948 Initialization
// ═══════════════════════════════════════════════════════════════════════════════

bool ImuDriverNode::init_icm20948()
{
  // 1) Reset the device — retry with back-off because the ICM-20948 can
  //    NACK if the I2C bus was left in a bad state from a previous run.
  bool reset_ok = false;
  for (int attempt = 0; attempt < 5; ++attempt) {
    // Small delay before each attempt to let the bus settle
    std::this_thread::sleep_for(std::chrono::milliseconds(50 * (attempt + 1)));

    if (!select_bank(0)) { continue; }
    if (write_register(bank0::PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET)) {
      reset_ok = true;
      current_bank_ = 0xFF;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
      "ICM-20948 reset attempt %d/5 failed (NACK). Retrying...", attempt + 1);
    current_bank_ = 0xFF;  // Force bank re-selection on next attempt
  }
  if (!reset_ok) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to reset ICM-20948 after 5 attempts. Check wiring / power cycle the sensor.");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 2) Wake up, auto-select clock
  if (!write_register(bank0::PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to wake ICM-20948.");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 3) Verify WHO_AM_I
  uint8_t who_am_i = 0;
  bool who_ok = false;
  for (int i = 0; i < 5; ++i) {
    if (read_register(bank0::WHO_AM_I, who_am_i)) {
      who_ok = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!who_ok) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read WHO_AM_I after 5 attempts.");
  return false;
}
  if (who_am_i != ICM20948_WHO_AM_I_VAL) {
    RCLCPP_ERROR(this->get_logger(),
      "WHO_AM_I mismatch: expected 0x%02X, got 0x%02X. Wrong device or wiring issue.",
      ICM20948_WHO_AM_I_VAL, who_am_i);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "ICM-20948 detected (WHO_AM_I = 0x%02X).", who_am_i);
  current_bank_ = 0xFF;  // force a real write to REG_BANK_SEL
  if (!select_bank(0)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // 4) Enable all accel + gyro axes
  if (!write_register(bank0::PWR_MGMT_2, PWR_MGMT_2_ALL_ENABLED)) return false;

  // 5) Configure gyro (Bank 2)
  if (!select_bank(2)) return false;
  uint8_t gyro_config = get_gyro_fs_bits() | GYRO_DLPF_ENABLE | GYRO_DLPF_CFG_3;
  if (!write_register(bank2::GYRO_CONFIG_1, gyro_config)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure gyroscope.");
    return false;
  }
  // Gyro sample rate divider = 0 → ODR = 1125 / (1 + 0) = 1125 Hz (internal)
  if (!write_register(bank2::GYRO_SMPLRT_DIV, 0x00)) return false;

  // 6) Configure accel (Bank 2)
  uint8_t accel_config = get_accel_fs_bits() | ACCEL_DLPF_ENABLE | ACCEL_DLPF_CFG_3;
  if (!write_register(bank2::ACCEL_CONFIG, accel_config)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure accelerometer.");
    return false;
  }
  // Accel sample rate divider = 0 → ODR = 1125 / (1 + 0) = 1125 Hz
  if (!write_register(bank2::ACCEL_SMPLRT_DIV_1, 0x00)) return false;
  if (!write_register(bank2::ACCEL_SMPLRT_DIV_2, 0x00)) return false;

  // 7) Enable I2C Master mode for Magnetometer (No more Bypass)
  if (enable_magnetometer_) {
    // First, reset the I2C master to clear any leftover state
    if (!select_bank(0)) return false;
    if (!write_register(bank0::USER_CTRL, USER_CTRL_I2C_MST_RST)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Configure the master clock BEFORE enabling
    if (!select_bank(3)) return false;
    if (!write_register(bank3::I2C_MST_CTRL, I2C_MST_CLK_345KHZ)) return false;

    // Now enable it
    if (!select_bank(0)) return false;
    if (!write_register(bank0::USER_CTRL, USER_CTRL_I2C_MST_EN)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Return to Bank 0 for data reads
  if (!select_bank(0)) return false;

  RCLCPP_INFO(this->get_logger(), "ICM-20948 configured: gyro ±%d dps, accel ±%d g.",
    gyro_range_dps_, accel_range_g_);
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// AK09916 Magnetometer Initialization (via I2C Master)
// ═══════════════════════════════════════════════════════════════════════════════

bool ImuDriverNode::init_ak09916()
{
  // 1) Soft reset
  if (!i2c_master_write(AK09916_ADDR, ak09916::CNTL3, AK09916_SOFT_RESET)) {
    RCLCPP_WARN(this->get_logger(), "AK09916 soft reset write failed (may be OK).");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 2) Verify device ID
  uint8_t device_id = 0;
  if (!i2c_master_read(AK09916_ADDR, ak09916::WIA2, device_id)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read AK09916 device ID.");
    return false;
  }
  if (device_id != AK09916_DEVICE_ID) {
    RCLCPP_ERROR(this->get_logger(),
      "AK09916 ID mismatch: expected 0x%02X, got 0x%02X. ",
      AK09916_DEVICE_ID, device_id);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "AK09916 magnetometer detected (ID = 0x%02X).", device_id);

  // 3) Set continuous measurement mode 4 (100 Hz)
  if (!i2c_master_write(AK09916_ADDR, ak09916::CNTL2, AK09916_MODE_CONT_100HZ)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set AK09916 continuous mode.");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // 4) Configure I2C Master SLV0 to continuously read 8 bytes from AK09916 ST1 (0x10)
  if (!select_bank(3)) return false;
  if (!write_register(bank3::I2C_SLV0_ADDR, AK09916_ADDR | 0x80)) return false; // Read flag
  if (!write_register(bank3::I2C_SLV0_REG, ak09916::ST1)) return false; // Start reg
  if (!write_register(bank3::I2C_SLV0_CTRL, 0x80 | 8)) return false; // Enable, 8 bytes

  // Back to Bank 0
  if (!select_bank(0)) return false;

  RCLCPP_INFO(this->get_logger(), "AK09916 configured: continuous 100 Hz mode (I2C Master).");
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Gyro Bias Calibration
// ═══════════════════════════════════════════════════════════════════════════════

void ImuDriverNode::calibrate_gyro()
{
  double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
  double ax, ay, az, gx, gy, gz;
  int valid_samples = 0;

  // Make sure we're in Bank 0 for data reads
  select_bank(0);

  for (int i = 0; i < calibration_samples_; ++i) {
    if (read_accel_gyro(ax, ay, az, gx, gy, gz)) {
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
      ++valid_samples;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (valid_samples > 0) {
    gyro_bias_x_ = sum_gx / valid_samples;
    gyro_bias_y_ = sum_gy / valid_samples;
    gyro_bias_z_ = sum_gz / valid_samples;
  } else {
    RCLCPP_WARN(this->get_logger(), "Gyro calibration got 0 valid samples. Bias stays zero.");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Publishers, Timer, Services
// ═══════════════════════════════════════════════════════════════════════════════

void ImuDriverNode::setup_publishers()
{
  // Raw IMU (no orientation) — consumed by imu_filter_madgwick
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

  if (enable_magnetometer_) {
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
  }

  if (publish_temperature_) {
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
  }
}

void ImuDriverNode::setup_timer()
{
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(period, std::bind(&ImuDriverNode::read_and_publish, this));
}

void ImuDriverNode::setup_services()
{
  // Service to recalibrate gyro at runtime (call with: ros2 service call /imu/calibrate std_srvs/srv/Trigger)
  calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/imu/calibrate",
    std::bind(&ImuDriverNode::calibrate_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

// ═══════════════════════════════════════════════════════════════════════════════
// Main Read & Publish Loop
// ═══════════════════════════════════════════════════════════════════════════════
void ImuDriverNode::read_and_publish()
{
  if (!select_bank(0)) return;

  // One 22-byte read: accel(6) + gyro(6) + temp(2) + ext_slv_mag(8)
  // Registers 0x2D through 0x42 are contiguous
  const size_t read_len = enable_magnetometer_ ? 22 : 14;  // 14 = accel(6)+gyro(6)+temp(2)
  uint8_t buf[22];
  if (!read_registers(bank0::ACCEL_XOUT_H, buf, read_len)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Failed to read sensor data.");
    return;
  }

  auto now = this->now();

  // Parse accel (buf[0..5]) — big-endian
  int16_t raw_ax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  int16_t raw_ay = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  int16_t raw_az = static_cast<int16_t>((buf[4] << 8) | buf[5]);

  // Parse gyro (buf[6..11]) — big-endian
  int16_t raw_gx = static_cast<int16_t>((buf[6]  << 8) | buf[7]);
  int16_t raw_gy = static_cast<int16_t>((buf[8]  << 8) | buf[9]);
  int16_t raw_gz = static_cast<int16_t>((buf[10] << 8) | buf[11]);

  // buf[12..13] = temp (skip or use if publish_temperature_ is true)

  
  // Convert and publish IMU
  double accel_sens = get_accel_sensitivity();
  double gyro_sens  = get_gyro_sensitivity();
  
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp    = now;
  imu_msg.header.frame_id = frame_id_;
  imu_msg.orientation_covariance[0] = -1.0;
  
  imu_msg.linear_acceleration.x = (raw_ax / accel_sens) * GRAVITY_MS2;
  imu_msg.linear_acceleration.y = (raw_ay / accel_sens) * GRAVITY_MS2;
  imu_msg.linear_acceleration.z = (raw_az / accel_sens) * GRAVITY_MS2;
  imu_msg.linear_acceleration_covariance[0] = accel_covariance_;
  imu_msg.linear_acceleration_covariance[4] = accel_covariance_;
  imu_msg.linear_acceleration_covariance[8] = accel_covariance_;
  
  imu_msg.angular_velocity.x = ((raw_gx / gyro_sens) * DEG_TO_RAD) - gyro_bias_x_;
  imu_msg.angular_velocity.y = ((raw_gy / gyro_sens) * DEG_TO_RAD) - gyro_bias_y_;
  imu_msg.angular_velocity.z = ((raw_gz / gyro_sens) * DEG_TO_RAD) - gyro_bias_z_;
  imu_msg.angular_velocity_covariance[0] = gyro_covariance_;
  imu_msg.angular_velocity_covariance[4] = gyro_covariance_;
  imu_msg.angular_velocity_covariance[8] = gyro_covariance_;

  imu_pub_->publish(imu_msg);

  // Publish mag if data ready and no overflow
   if (enable_magnetometer_) {
    uint8_t mag_st1 = buf[14];
    uint8_t mag_st2 = buf[21];
    if ((mag_st1 & 0x01) && !(mag_st2 & 0x08)) {
      int16_t raw_mx = static_cast<int16_t>(buf[15] | (buf[16] << 8));
      int16_t raw_my = static_cast<int16_t>(buf[17] | (buf[18] << 8));
      int16_t raw_mz = static_cast<int16_t>(buf[19] | (buf[20] << 8));
      
      auto mag_msg = sensor_msgs::msg::MagneticField();
      mag_msg.header.stamp    = now;
      mag_msg.header.frame_id = frame_id_;
      mag_msg.magnetic_field.x = raw_mx * MAG_SENSITIVITY_UT * UT_TO_TESLA;
      mag_msg.magnetic_field.y = raw_my * MAG_SENSITIVITY_UT * UT_TO_TESLA;
      mag_msg.magnetic_field.z = raw_mz * MAG_SENSITIVITY_UT * UT_TO_TESLA;
      mag_msg.magnetic_field_covariance[0] = mag_covariance_;
      mag_msg.magnetic_field_covariance[4] = mag_covariance_;
      mag_msg.magnetic_field_covariance[8] = mag_covariance_;
      mag_pub_->publish(mag_msg);
    }
  }

}

// ═══════════════════════════════════════════════════════════════════════════════
// I2C Low-Level Helpers
// ═══════════════════════════════════════════════════════════════════════════════

bool ImuDriverNode::write_register(uint8_t reg, uint8_t value)
{
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (i2c_smbus_write_byte_data(i2c_fd_, reg, value) >= 0) {
      return true;
    }
    usleep(500);
  }
  RCLCPP_ERROR(this->get_logger(), "I2C write failed: reg=0x%02X, val=0x%02X: %s",
    reg, value, strerror(errno));
  return false;
}

bool ImuDriverNode::read_register(uint8_t reg, uint8_t & value)
{
  for (int attempt = 0; attempt < 3; ++attempt) {
    int32_t result = i2c_smbus_read_byte_data(i2c_fd_, reg);
    if (result >= 0) {
      value = static_cast<uint8_t>(result);
      return true;
    }
    usleep(500);
  }
  RCLCPP_ERROR(this->get_logger(), "I2C read failed: reg=0x%02X: %s",
    reg, strerror(errno));
  return false;
}

bool ImuDriverNode::read_registers(uint8_t start_reg, uint8_t * buffer, size_t length)
{
  for (int attempt = 0; attempt < 3; ++attempt) {
    int32_t result = i2c_smbus_read_i2c_block_data(
      i2c_fd_, start_reg, static_cast<uint8_t>(length), buffer);
    if (result >= 0) {
      return true;
    }
    usleep(500);
  }
  return false;
}

// bool ImuDriverNode::write_register(uint8_t reg, uint8_t value)
// {
//   uint8_t buf[2] = {reg, value};
//   struct i2c_msg msg;
//   msg.addr  = i2c_address_;
//   msg.flags = 0;
//   msg.len   = 2;
//   msg.buf   = buf;

//   struct i2c_rdwr_ioctl_data rdwr;
//   rdwr.msgs  = &msg;
//   rdwr.nmsgs = 1;

//   for (int attempt = 0; attempt < 3; ++attempt) {
//     if (ioctl(i2c_fd_, I2C_RDWR, &rdwr) >= 0) {
//       return true;
//     }
//     usleep(200);
//   }

//   RCLCPP_ERROR(this->get_logger(), "I2C write failed: reg=0x%02X, val=0x%02X: %s",
//     reg, value, strerror(errno));
//   return false;
// }

// bool ImuDriverNode::read_register(uint8_t reg, uint8_t & value)
// {
//   struct i2c_msg msgs[2];
//   msgs[0].addr  = i2c_address_;
//   msgs[0].flags = 0;
//   msgs[0].len   = 1;
//   msgs[0].buf   = &reg;

//   msgs[1].addr  = i2c_address_;
//   msgs[1].flags = I2C_M_RD;
//   msgs[1].len   = 1;
//   msgs[1].buf   = &value;

//   struct i2c_rdwr_ioctl_data rdwr;
//   rdwr.msgs  = msgs;
//   rdwr.nmsgs = 2;

//   for (int attempt = 0; attempt < 3; ++attempt) {
//     if (ioctl(i2c_fd_, I2C_RDWR, &rdwr) >= 0) {
//       return true;  // ✅ success — return immediately, no delay
//     }
//     usleep(200);  // ✅ delay only between failed attempts
//   }

//   RCLCPP_ERROR(this->get_logger(), "I2C read failed: reg=0x%02X: %s",
//     reg, strerror(errno));
//   return false;
// }

// bool ImuDriverNode::read_registers(uint8_t start_reg, uint8_t * buffer, size_t length)
// {
//   struct i2c_msg msgs[2];
//   msgs[0].addr  = i2c_address_;
//   msgs[0].flags = 0;
//   msgs[0].len   = 1;
//   msgs[0].buf   = &start_reg;

//   msgs[1].addr  = i2c_address_;
//   msgs[1].flags = I2C_M_RD;
//   msgs[1].len   = static_cast<uint16_t>(length);
//   msgs[1].buf   = buffer;

//   struct i2c_rdwr_ioctl_data rdwr;
//   rdwr.msgs  = msgs;
//   rdwr.nmsgs = 2;

//   for (int attempt = 0; attempt < 3; ++attempt) {
//     if (ioctl(i2c_fd_, I2C_RDWR, &rdwr) >= 0) {
//       return true;
//     }
//     usleep(200);  // 200µs between retries — lets RP1 release the bus
//   }
//   return false;
// }

bool ImuDriverNode::select_bank(uint8_t bank)
{
  if (bank == current_bank_) return true;  // Already on this bank

  uint8_t bank_val = (bank & 0x03) << 4;
  if (!write_register(REG_BANK_SEL, bank_val)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to select bank %d.", bank);
    return false;
  }
  current_bank_ = bank;
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// I2C Master Helpers (via SLV4)
// ═══════════════════════════════════════════════════════════════════════════════

bool ImuDriverNode::i2c_master_write(uint8_t slave_addr, uint8_t reg, uint8_t value)
{
  if (!select_bank(3)) return false;

  if (!write_register(bank3::I2C_SLV4_ADDR, slave_addr)) return false;
  if (!write_register(bank3::I2C_SLV4_REG,  reg)) return false;
  if (!write_register(bank3::I2C_SLV4_DO,   value)) return false;
  if (!write_register(bank3::I2C_SLV4_CTRL, 0x80)) return false;

  // I2C_MST_STATUS lives in Bank 0 — switch there to poll
  if (!select_bank(0)) return false;

  for (int i = 0; i < 50; ++i) {
    uint8_t status = 0;
    if (read_register(bank0::I2C_MST_STATUS, status)) {
      if (status & 0x40) {                // SLV4_DONE
        if (status & 0x10) return false;  // SLV4_NACK — slave didn't ack
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

bool ImuDriverNode::i2c_master_read(uint8_t slave_addr, uint8_t reg, uint8_t & value)
{
  if (!select_bank(3)) return false;

  if (!write_register(bank3::I2C_SLV4_ADDR, slave_addr | 0x80)) return false;  // read bit
  if (!write_register(bank3::I2C_SLV4_REG,  reg)) return false;
  if (!write_register(bank3::I2C_SLV4_CTRL, 0x80)) return false;

  // Poll status in Bank 0
  if (!select_bank(0)) return false;
  bool done = false;
  for (int i = 0; i < 50; ++i) {
    uint8_t status = 0;
    if (read_register(bank0::I2C_MST_STATUS, status)) {
      if (status & 0x40) {
        if (status & 0x10) return false;  // NACK from slave
        done = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!done) return false;

  // Result is in I2C_SLV4_DI (Bank 3, 0x17)
  if (!select_bank(3)) return false;
  return read_register(bank3::I2C_SLV4_DI, value);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sensor Reading
// ═══════════════════════════════════════════════════════════════════════════════

bool ImuDriverNode::read_accel_gyro(
  double & ax, double & ay, double & az,
  double & gx, double & gy, double & gz)
{
  // Ensure Bank 0
  if (!select_bank(0)) return false;

  // Read 12 bytes: ACCEL_XOUT_H (0x2D) through GYRO_ZOUT_L (0x38)
  uint8_t buf[12];
  if (!read_registers(bank0::ACCEL_XOUT_H, buf, 12)) {
    return false;
  }

  // Combine high/low bytes (big-endian, signed)
  int16_t raw_ax = static_cast<int16_t>((buf[0]  << 8) | buf[1]);
  int16_t raw_ay = static_cast<int16_t>((buf[2]  << 8) | buf[3]);
  int16_t raw_az = static_cast<int16_t>((buf[4]  << 8) | buf[5]);
  int16_t raw_gx = static_cast<int16_t>((buf[6]  << 8) | buf[7]);
  int16_t raw_gy = static_cast<int16_t>((buf[8]  << 8) | buf[9]);
  int16_t raw_gz = static_cast<int16_t>((buf[10] << 8) | buf[11]);

  double accel_sens = get_accel_sensitivity();
  double gyro_sens  = get_gyro_sensitivity();

  // Convert accel to m/s²
  ax = (raw_ax / accel_sens) * GRAVITY_MS2;
  ay = (raw_ay / accel_sens) * GRAVITY_MS2;
  az = (raw_az / accel_sens) * GRAVITY_MS2;

  // Convert gyro to rad/s
  gx = (raw_gx / gyro_sens) * DEG_TO_RAD;
  gy = (raw_gy / gyro_sens) * DEG_TO_RAD;
  gz = (raw_gz / gyro_sens) * DEG_TO_RAD;

  return true;
}

bool ImuDriverNode::read_magnetometer(double & mx, double & my, double & mz)
{
  // Ensure Bank 0
  if (!select_bank(0)) return false;

  // Read 8 bytes from EXT_SLV_SENS_DATA_00 (0x3B)
  // These are continuously populated by the I2C Master SLV0 reading from AK09916 ST1 (0x10)
  // Layout: ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2
  uint8_t buf[8];
  if (!read_registers(bank0::EXT_SLV_SENS_DATA_00, buf, 8)) return false;

  // Check data ready (ST1 bit 0)
  uint8_t st1 = buf[0];
  if (!(st1 & 0x01)) {
    return false;  // Data not ready yet — skip this cycle
  }

  // Check for magnetic sensor overflow (ST2 bit 3)
  uint8_t st2 = buf[7];
  if (st2 & 0x08) {
    return false;  // Overflow — discard reading
  }

  // Combine bytes (little-endian for AK09916!)
  int16_t raw_mx = static_cast<int16_t>(buf[1] | (buf[2] << 8));
  int16_t raw_my = static_cast<int16_t>(buf[3] | (buf[4] << 8));
  int16_t raw_mz = static_cast<int16_t>(buf[5] | (buf[6] << 8));

  // Convert to Tesla (sensor_msgs/MagneticField uses Tesla)
  // Raw → µT (× 0.15) → Tesla (× 1e-6)
  mx = raw_mx * MAG_SENSITIVITY_UT * UT_TO_TESLA;
  my = raw_my * MAG_SENSITIVITY_UT * UT_TO_TESLA;
  mz = raw_mz * MAG_SENSITIVITY_UT * UT_TO_TESLA;

  return true;
}

bool ImuDriverNode::read_temperature(double & temp_c)
{
  if (!select_bank(0)) return false;

  uint8_t buf[2];
  if (!read_registers(bank0::TEMP_OUT_H, buf, 2)) return false;

  int16_t raw_temp = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  temp_c = (raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET;

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Service: Runtime Recalibration
// ═══════════════════════════════════════════════════════════════════════════════

void ImuDriverNode::calibrate_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Recalibrating gyro...");
  calibrate_gyro();
  response->success = true;
  response->message =
    "Gyro bias: x=" + std::to_string(gyro_bias_x_) +
    ", y=" + std::to_string(gyro_bias_y_) +
    ", z=" + std::to_string(gyro_bias_z_) + " rad/s";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sensitivity / Config Helpers
// ═══════════════════════════════════════════════════════════════════════════════

double ImuDriverNode::get_gyro_sensitivity() const
{
  switch (gyro_range_dps_) {
    case 250:  return GYRO_SENSITIVITY_250DPS;
    case 500:  return GYRO_SENSITIVITY_500DPS;
    case 1000: return GYRO_SENSITIVITY_1000DPS;
    case 2000: return GYRO_SENSITIVITY_2000DPS;
    default:
      RCLCPP_WARN(rclcpp::get_logger("icm20948"),
        "Invalid gyro range %d dps. Defaulting to 500.", gyro_range_dps_);
      return GYRO_SENSITIVITY_500DPS;
  }
}

double ImuDriverNode::get_accel_sensitivity() const
{
  switch (accel_range_g_) {
    case 2:  return ACCEL_SENSITIVITY_2G;
    case 4:  return ACCEL_SENSITIVITY_4G;
    case 8:  return ACCEL_SENSITIVITY_8G;
    case 16: return ACCEL_SENSITIVITY_16G;
    default:
      RCLCPP_WARN(rclcpp::get_logger("icm20948"),
        "Invalid accel range %d g. Defaulting to 4.", accel_range_g_);
      return ACCEL_SENSITIVITY_4G;
  }
}

uint8_t ImuDriverNode::get_gyro_fs_bits() const
{
  switch (gyro_range_dps_) {
    case 250:  return GYRO_FS_250DPS;
    case 500:  return GYRO_FS_500DPS;
    case 1000: return GYRO_FS_1000DPS;
    case 2000: return GYRO_FS_2000DPS;
    default:   return GYRO_FS_500DPS;
  }
}

uint8_t ImuDriverNode::get_accel_fs_bits() const
{
  switch (accel_range_g_) {
    case 2:  return ACCEL_FS_2G;
    case 4:  return ACCEL_FS_4G;
    case 8:  return ACCEL_FS_8G;
    case 16: return ACCEL_FS_16G;
    default: return ACCEL_FS_4G;
  }
}

}  // namespace imu_sensor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imu_sensor::ImuDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}