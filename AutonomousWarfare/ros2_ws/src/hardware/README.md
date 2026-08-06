# hardware

## Purpose
Manage low-level hardware interfaces including motor drivers and the Arduino serial bridge.

## Logic
Translates high-level velocity commands into PWM signals for the L298N motor driver and manages serial communication with the Arduino for steering.

## Data Flow
- **Input:** `geometry_msgs/Twist` on `/cmd_vel`.
- **Output:** PWM signals to GPIO, serial commands to `/dev/ttyACM0`.
