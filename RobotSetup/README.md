# Eye-To-Zion - Autonomous Robot Combat

An autonomous, ROS2-based robotic platform designed for combat/navigation scenarios. It features computer vision (YOLO), audio impact detection (AI audio), lidar-based SLAM, and tactical behavior trees for autonomous decision-making. 

## Hardware Required
See [HARDWARE.md](HARDWARE.md) for the full Bill of Materials.

**Quick Summary of Major Components:**
- Raspberry Pi 5 (Main Brain)
- Arduino Uno/Nano (Steering/Flag Controller)
- L298N Motor Driver + 2x DC Motors
- YDLidar + ICM20948 IMU
- Pi Camera + ICS-43434 I2S Microphone

## Quick Start

### 1. Wire the hardware
See [WIRING.md](WIRING.md) for the complete wiring guide.

### 2. Set up the Pi
On a fresh Raspberry Pi OS (Bookworm 64-bit) for Pi 5:
```bash
git clone https://github.com/yahav1221/Eye-To-Zion---Autonomous-Robot-Combat.git
cd Eye-To-Zion---Autonomous-Robot-Combat-RPI
chmod +x setup.sh
./setup.sh
sudo reboot
```

### 3. Run
The setup script creates a virtual environment and a systemd service (`robot.service`). 
To run manually:
```bash
source venv/bin/activate
./start_system.sh
```
(Or it will start automatically on boot if you enabled `robot.service` via systemd.)

## Project Structure
- `AutonomousWarfare/ros2_ws/`: Core ROS2 Humble workspace containing all functional packages.
  - `src/ai_audio/`: CNN-based chassis impact detection using microphone.
  - `src/ai_vision/`: YOLO inference on camera streams.
  - `src/hardware/`: C++ Motor Driver (L298N + Arduino Serial), Odometry, PID controllers.
  - `src/localization/`: EKF and SLAM configurations.
  - `src/navigation/`: Nav2 configurations and launch files.
  - `src/tactical_brain/`: Behavior trees for high-level tactical decision making.
  - `src/robot_stats/`: Health and ammo management nodes.
  - `src/sensor_fusion_pkg/`: Fusion of sensor data.
  - `src/telemetry_data/`: InfluxDB/Telegraf bridges for dashboard metrics.
- `telegraf.conf` / `docker-compose.yml`: Infrastructure for telemetry and metrics.
- `setup.sh`: Automated Pi 5 configuration script.

## Configuration
- **Hardware Parameters**: Modify `AutonomousWarfare/ros2_ws/src/hardware/ros2_control.xacro` to change GPIO pins or PWM channels.
- **Nav2 Settings**: Modify `AutonomousWarfare/ros2_ws/src/navigation/config/nav2.yaml` for robot dimensions, max speeds, and costmaps.
- **Lidar Settings**: Modify configs in `AutonomousWarfare/ros2_ws/src/ydlidar_ros2_driver/params/` according to your specific YDLidar model.

## Troubleshooting
- **I2C Device Not Found**: Check WIRING.md. Ensure `SDA` and `SCL` are properly connected and `sudo i2cdetect -y 1` shows `0x68`.
- **Motors Not Moving**: Verify the L298N has external battery power and shares a ground with the Raspberry Pi. Ensure the `pigpiod` service is running or `/sys/class/pwm` is accessible.
- **Serial Permission Denied**: Ensure your user is in the `dialout` group (`sudo usermod -a -G dialout $USER`), then logout and login.
- **Camera Not Found**: Ensure the ribbon cable is seated correctly and you enabled camera support via `raspi-config`. Run `libcamera-hello` to test.
