#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2023 University of Leeds
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField

from icm20948 import ICM20948

PUBLISH_RATE = 20  # per second
PUBLISH_INTERVAL_S = 1 / PUBLISH_RATE
SLOW_PUBLISH_INTERVAL_S = 1.0


class ImuNode(Node):
    def __init__(self, imu):
        # Initialise the Node.
        super().__init__("imu_icm20948")
        self._imu = imu
        self.get_logger().info("IMU has started!")
        # Set up Publishers
        self.imu_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 10)
        self.imu_temp_publisher_ = self.create_publisher(Temperature, "imu/temp", 10)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 10)
        # Timers.
        self.data_timer_ = self.create_timer(PUBLISH_INTERVAL_S, self._publish_all)
        self.temperature_timer_ = self.create_timer(SLOW_PUBLISH_INTERVAL_S, self._publish_temperature)

    def _setup_imu(self):
        """
        Set up the IMU.
        Note: Most of the default settings are fine but this is provided so that
        the user can change settings if they want to.
        """
        # This sequence is copied from the library initialisation code.
        # Write to bank 2 to configure the settings.
        self._imu.bank(2)

        # Set sample rate in Hz.  Default = 100Hz, max = 225Hz.
        self._imu.set_gyro_sample_rate(100)
        # Set full scale range of gyroscope in degrees per second.
        # Valid values are 250, 500, 1000, 2000.
        self._imu.set_gyro_full_scale(250)
        # Read the code and the datasheet to see what this does.
        # self._imu.set_gyro_low_pass(enabled=True, mode=5)

        # Set sample rate in Hz.  Default = 125Hz, max = 1kHz.
        self._imu.set_accelerometer_sample_rate(125)
        # Set full scale range of accelerometer in g.
        # Valid values are 2, 4, 8, 16.
        self._imu.set_accelerometer_full_scale(4)
        # Read the code and the datasheet to see what this does.
        # self._imu.set_accelerometer_low_pass(enabled=True, mode=5)

        # Return to normal mode.
        self._imu.bank(0)

    def _publish_all(self):
        # Get all readings.
        self._publish_raw()
        self._publish_magnetic()

    def _publish_raw(self):
        msg = Imu()
        ax, ay, az, gx, gy, gz = self._imu.read_accelerometer_gyro_data()
        # Note: raw gyroscope data is reported in degrees per second.
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        # Note: raw acceleration is reported in degrees per second.
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        # Publish the message.
        self.imu_raw_publisher_.publish(msg)

    def _publish_magnetic(self):
        msg = MagneticField()
        x, y, z = self._imu.read_magnetometer_data()
        # Convert from micro-Teslas to Teslas.
        msg.magnetic_field.x = float(x) / 1000.0
        msg.magnetic_field.y = float(y) / 1000.0
        msg.magnetic_field.z = float(z) / 1000.0
        # Publish the message.
        self.imu_mag_publisher_.publish(msg)

    def _publish_temperature(self):
        msg = Temperature()
        temperature_deg_c = self._imu.read_temperature()
        msg.temperature = float(temperature_deg_c)
        self.get_logger().info("pub imu_temp" + str(msg.temperature))
        # Publish the message.
        self.imu_temp_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    imu = ICM20948()
    node = ImuNode(imu)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
