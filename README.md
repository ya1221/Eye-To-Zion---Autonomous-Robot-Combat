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
The setup script configures the host dependencies and hardware interfaces. 
To run the robot manually via Docker:
```bash
cd AutonomousWarfare
./start_system.sh
```
(Or it will start automatically on boot if you enabled `robot.service` via systemd.)

## Project Structure
- `AutonomousWarfare/ros2_ws/`: Core ROS2 Humble workspace containing all functional packages.
  - `src/ai_audio/`: CNN-based chassis impact detection using microphone.
    - **Code Links:**
      - [Converts audio to Log-Mel spectrograms to detect real-time physical impacts using only a microphone](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/ai_audio/ai_audio/audio_processor_node.py#L147C5-L181C1)
      - [Samples audio right before impact to catch the hit peak and avoid duplicate triggers](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/ai_audio/ai_audio/trigger.py#L24C5-L53C36)
  - `src/ai_vision/`: YOLO inference on camera streams.
    - **Code Links:**
      -  [Ranks targets by size and stability to enable prioritized tactical engagement](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/ai_vision/ai_vision/ai_inference_node.py#L171C5-L194C1)
      -  [Maintains target lock through temporary occlusions](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/ai_vision/ai_vision/ai_inference_node.py#L287C5-L318C40)
      - [Converts pixels to turret firing angles](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/ai_vision/ai_vision/ai_inference_node.py#L244C17-L248C37)
  - `src/hardware/`: C++ Motor Driver (L298N + Arduino Serial), Odometry, PID controllers.
    - **Code Links:**
      - [Motor driver class - helper method to write configuration values to PWM sysfs files ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/hardware/src/motor_driver.cpp#L11C1-L17C2)
  - `src/localization/`: EKF and SLAM configurations.
    - **Code Links:**
      - [Localization launch file - Dynamic loading of parameters for different YDLidar models ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/localization/launch/launch.py#L13C5-L24C7)
  - `src/navigation/`: Nav2 configurations and launch files.
    - **Code Links:**
      - *(Add links here)*
  - `src/tactical_brain/`: Behavior trees for high-level tactical decision making.
    - **Code Links:**
      - *(Add links here)*
  - `src/robot_stats/`: Health and ammo management nodes.
    - **Code Links:**
      - [Robot stats class - receiving robot internal states updates from other nodes ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/robot_stats/src/robot_stats.cpp#L5C1-L31C2)
  - `src/sensor_fusion_pkg/`: Fusion of sensor data.
    - **Code Links:**
      - *(Add links here)*
  - `src/telemetry_data/`: InfluxDB/Telegraf bridges for dashboard metrics.
    - **Code Links:**
      - [Telemetry data class - sending telemetry data to Telegraf ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/telemetry_data/src/telemetry_sender.cpp#L39C1-L50C2)
      - [Telegraf node class - receiving telemetry data from other nodes ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/telemetry_data/src/telegraf_bridge.cpp#L8C1-L29C2)
  - `src/robot_description/`: Robot description and URDF files.
    - **Code Links:**
      - [Robot description URDF file - URDF base file of the robot ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/robot_description/urdf/robot_urdf.xacro#L1C1-L7C9)
  - `src/robot_bringup/`: Robot bringup files.
    - **Code Links:**
      - [Robot bringup launch file - launch all the packages together ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/robot_bringup/launch/launch.py#L80C5-L94C7)
  - `src/shooting/`: Shooting mechanism node.
    - **Code Links:**
      - [Shooting node class - shooting mechanism node ](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/AutonomousWarfare/ros2_ws/src/shooting/src/shooting_node.cpp#L159C1-L164C2)
-  `Forward_Command_Post`
  - `src/overhead_tracker/`: 
    - **Code Links:**
      - ([Corrects camera perspective to a flat arena grid for precise distance measurements in mm](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/Forward_Command_Post/ros2_ws/src/overhead_tracker/overhead_tracker/overhead_tracker.py#L213C5-L224C37))
    - **Code Links:**
      - ([Tracks the robot body when markers are hidden (with Lucas-Kanade-Tomasi) to prevent dropouts on the tactical map](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/Forward_Command_Post/ros2_ws/src/overhead_tracker/overhead_tracker/overhead_tracker.py#L262C5-L293C47))
    - **Code Links:**
      - ([Assigning teams using ID modulo (rid % CNT_TEAM) enables dynamic support for N teams without code changes](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/Forward_Command_Post/ros2_ws/src/overhead_tracker/overhead_tracker/overhead_tracker.py#L231C1-L231C48))
-  `Offboard`
  - `ai_audio/pipeline/training/`:
    - **Code Links:**
      - ([ImpactCNN uses 3 convolution blocks ending with AdaptiveAvgPool2d(1) to maintain a fixed classification layer regardless of spectrogram dimensions](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/Offboard/ai_audio/pipeline/training/kaggle_train.ipynb#In[9]))
  - `ai_audio/pipeline/preprocessing/`: 
    - **Code Links:**
      - ([Isolates impacts from long audio by detecting energy peaks with a refractory period to prevent duplicates](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/Offboard/ai_audio/pipeline/preprocessing/make_features.py#L40C1-L64C28))
  - `ai_vision/`: 
    - **Code Links:**
      - ([Augments light and scale during training to prevent overfitting and ensure robust field detection](https://github.com/ya1221/Eye-To-Zion---Autonomous-Robot-Combat/blob/main/Offboard/ai_vision/eye-to-zion-ai-vision.ipynb#In[3]))
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
