#pragma once

#include <cstdint>

namespace imu_sensor
{

// ─── I2C Addresses ───────────────────────────────────────────────────────────
constexpr uint8_t ICM20948_ADDR_LOW  = 0x68;  // AD0 pin LOW (default)
constexpr uint8_t ICM20948_ADDR_HIGH = 0x69;  // AD0 pin HIGH
constexpr uint8_t AK09916_ADDR       = 0x0C;  // Magnetometer (accessed via bypass)

// ─── Device IDs ──────────────────────────────────────────────────────────────
constexpr uint8_t ICM20948_WHO_AM_I_VAL = 0xEA;
constexpr uint8_t AK09916_DEVICE_ID     = 0x09;

// ─── Bank Selection Register (same address in all banks) ─────────────────────
constexpr uint8_t REG_BANK_SEL = 0x7F;  // Bits [5:4] = bank number (0-3)

// ═══ BANK 0 REGISTERS ═══════════════════════════════════════════════════════
namespace bank0
{
  constexpr uint8_t WHO_AM_I       = 0x00;
  constexpr uint8_t USER_CTRL      = 0x03;
  constexpr uint8_t LP_CONFIG      = 0x05;
  constexpr uint8_t PWR_MGMT_1     = 0x06;
  constexpr uint8_t PWR_MGMT_2     = 0x07;
  constexpr uint8_t INT_PIN_CFG    = 0x0F;
  constexpr uint8_t INT_ENABLE     = 0x10;
  constexpr uint8_t INT_ENABLE_1   = 0x11;
  constexpr uint8_t INT_STATUS     = 0x19;
  constexpr uint8_t INT_STATUS_1   = 0x1A;
  constexpr uint8_t ACCEL_XOUT_H   = 0x2D;
  constexpr uint8_t ACCEL_XOUT_L   = 0x2E;
  constexpr uint8_t ACCEL_YOUT_H   = 0x2F;
  constexpr uint8_t ACCEL_YOUT_L   = 0x30;
  constexpr uint8_t ACCEL_ZOUT_H   = 0x31;
  constexpr uint8_t ACCEL_ZOUT_L   = 0x32;
  constexpr uint8_t GYRO_XOUT_H    = 0x33;
  constexpr uint8_t GYRO_XOUT_L    = 0x34;
  constexpr uint8_t GYRO_YOUT_H    = 0x35;
  constexpr uint8_t GYRO_YOUT_L    = 0x36;
  constexpr uint8_t GYRO_ZOUT_H    = 0x37;
  constexpr uint8_t GYRO_ZOUT_L    = 0x38;
  constexpr uint8_t TEMP_OUT_H     = 0x39;
  constexpr uint8_t TEMP_OUT_L     = 0x3A;
}  // namespace bank0

// PWR_MGMT_1 bits
constexpr uint8_t PWR_MGMT_1_DEVICE_RESET = 0x80;
constexpr uint8_t PWR_MGMT_1_SLEEP        = 0x40;
constexpr uint8_t PWR_MGMT_1_LP_EN        = 0x20;
constexpr uint8_t PWR_MGMT_1_TEMP_DIS     = 0x08;
constexpr uint8_t PWR_MGMT_1_CLKSEL_AUTO  = 0x01;  // Auto-select best clock

// USER_CTRL bits
constexpr uint8_t USER_CTRL_DMP_EN    = 0x80;
constexpr uint8_t USER_CTRL_FIFO_EN   = 0x40;
constexpr uint8_t USER_CTRL_I2C_MST_EN = 0x20;
constexpr uint8_t USER_CTRL_I2C_IF_DIS = 0x10;  // Disable I2C, use SPI only
constexpr uint8_t USER_CTRL_DMP_RST    = 0x08;
constexpr uint8_t USER_CTRL_SRAM_RST   = 0x04;
constexpr uint8_t USER_CTRL_I2C_MST_RST = 0x02;

// INT_PIN_CFG bits
constexpr uint8_t INT_PIN_CFG_BYPASS_EN = 0x02;  // I2C bypass mode for mag

// PWR_MGMT_2 bits (0 = enabled, 1 = disabled for each axis)
constexpr uint8_t PWR_MGMT_2_ALL_ENABLED = 0x00;  // All accel + gyro axes on

// ═══ BANK 2 REGISTERS ═══════════════════════════════════════════════════════
namespace bank2
{
  constexpr uint8_t GYRO_SMPLRT_DIV   = 0x00;
  constexpr uint8_t GYRO_CONFIG_1     = 0x01;
  constexpr uint8_t GYRO_CONFIG_2     = 0x02;
  constexpr uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;  // MSB [3:0]
  constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;  // LSB [7:0]
  constexpr uint8_t ACCEL_CONFIG       = 0x14;
  constexpr uint8_t ACCEL_CONFIG_2     = 0x15;
}  // namespace bank2

// GYRO_CONFIG_1: bits [2:1] = FS_SEL, bit [0] = FCHOICE (1=enable DLPF)
// DLPF bits [5:3] select bandwidth when FCHOICE=1
constexpr uint8_t GYRO_FS_250DPS  = (0x00 << 1);  // ±250 dps
constexpr uint8_t GYRO_FS_500DPS  = (0x01 << 1);  // ±500 dps
constexpr uint8_t GYRO_FS_1000DPS = (0x02 << 1);  // ±1000 dps
constexpr uint8_t GYRO_FS_2000DPS = (0x03 << 1);  // ±2000 dps
constexpr uint8_t GYRO_DLPF_ENABLE = 0x01;         // Enable DLPF
constexpr uint8_t GYRO_DLPF_CFG_6  = (0x06 << 3);  // BW ~5.7 Hz (clean, some latency)
constexpr uint8_t GYRO_DLPF_CFG_3  = (0x03 << 3);  // BW ~23.9 Hz (good default)
constexpr uint8_t GYRO_DLPF_CFG_0  = (0x00 << 3);  // BW ~196.6 Hz (fast, more noise)

// ACCEL_CONFIG: bits [2:1] = FS_SEL, bit [0] = FCHOICE (1=enable DLPF)
constexpr uint8_t ACCEL_FS_2G  = (0x00 << 1);  // ±2g
constexpr uint8_t ACCEL_FS_4G  = (0x01 << 1);  // ±4g
constexpr uint8_t ACCEL_FS_8G  = (0x02 << 1);  // ±8g
constexpr uint8_t ACCEL_FS_16G = (0x03 << 1);  // ±16g
constexpr uint8_t ACCEL_DLPF_ENABLE = 0x01;
constexpr uint8_t ACCEL_DLPF_CFG_3  = (0x03 << 3);  // BW ~23.5 Hz (good default)

// Sensitivity scale factors (LSB per unit)
constexpr double GYRO_SENSITIVITY_250DPS  = 131.0;    // LSB/dps
constexpr double GYRO_SENSITIVITY_500DPS  = 65.5;     // LSB/dps
constexpr double GYRO_SENSITIVITY_1000DPS = 32.8;     // LSB/dps
constexpr double GYRO_SENSITIVITY_2000DPS = 16.4;     // LSB/dps

constexpr double ACCEL_SENSITIVITY_2G  = 16384.0;     // LSB/g
constexpr double ACCEL_SENSITIVITY_4G  = 8192.0;      // LSB/g
constexpr double ACCEL_SENSITIVITY_8G  = 4096.0;      // LSB/g
constexpr double ACCEL_SENSITIVITY_16G = 2048.0;      // LSB/g

// Temperature conversion
constexpr double TEMP_SENSITIVITY = 333.87;  // LSB/°C
constexpr double TEMP_OFFSET      = 21.0;    // °C at raw=0

// Gravity in m/s² (for converting g to m/s²)
constexpr double GRAVITY_MS2 = 9.80665;

// Degrees to radians (gyro outputs dps, ROS expects rad/s)
constexpr double DEG_TO_RAD = 0.017453292519943295;

// ═══ AK09916 MAGNETOMETER REGISTERS ═════════════════════════════════════════
namespace ak09916
{
  constexpr uint8_t WIA1  = 0x00;  // Company ID (should be 0x48)
  constexpr uint8_t WIA2  = 0x01;  // Device ID (should be 0x09)
  constexpr uint8_t ST1   = 0x10;  // Status 1: bit 0 = DRDY (data ready)
  constexpr uint8_t HXL   = 0x11;  // Mag X low byte
  constexpr uint8_t HXH   = 0x12;
  constexpr uint8_t HYL   = 0x13;
  constexpr uint8_t HYH   = 0x14;
  constexpr uint8_t HZL   = 0x15;
  constexpr uint8_t HZH   = 0x16;
  constexpr uint8_t ST2   = 0x18;  // Status 2: bit 3 = overflow. Must read to end cycle.
  constexpr uint8_t CNTL2 = 0x31;  // Control 2: operation mode
  constexpr uint8_t CNTL3 = 0x32;  // Control 3: bit 0 = soft reset
}  // namespace ak09916

// AK09916 operation modes (written to CNTL2)
constexpr uint8_t AK09916_MODE_POWER_DOWN = 0x00;
constexpr uint8_t AK09916_MODE_SINGLE     = 0x01;
constexpr uint8_t AK09916_MODE_CONT_10HZ  = 0x02;
constexpr uint8_t AK09916_MODE_CONT_20HZ  = 0x04;
constexpr uint8_t AK09916_MODE_CONT_50HZ  = 0x06;
constexpr uint8_t AK09916_MODE_CONT_100HZ = 0x08;
constexpr uint8_t AK09916_SOFT_RESET       = 0x01;  // Write to CNTL3

// AK09916 sensitivity: 0.15 µT/LSB
constexpr double MAG_SENSITIVITY_UT = 0.15;  // µT per LSB

// Convert µT to Tesla for sensor_msgs/MagneticField (which uses Tesla)
constexpr double UT_TO_TESLA = 1e-6;

}  // namespace icm20948

