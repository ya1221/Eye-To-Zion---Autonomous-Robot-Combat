# RobotSetup Directory Reference

This document explains every file in `RobotSetup/` (except `README.md`) and how they fit together to provision a Raspberry Pi 5 for the Eye-To-Zion robot.

## File Overview

| File | Type | Purpose |
|---|---|---|
| `install_robot.sh` | Script (entry point) | Orchestrates the full install: runs `setup.sh`, then `verify_hardware.py`, then prints next steps. |
| `setup.sh` | Script | Does the actual provisioning: apt packages, hardware interfaces, `pigpiod`, boot config, systemd service, permissions. |
| `verify_hardware.py` | Script | Post-install diagnostic that checks Python deps, I2C/Serial/PWM/GPIO interfaces, the IMU, camera, and mic. |
| `config.txt.additions` | Config snippet | Boot-time hardware overlays appended to `/boot/firmware/config.txt` by `setup.sh`. |
| `robot.service` | systemd unit | Defines the background service that auto-starts the robot software on boot. |
| `.env.example` | Config template | Template for environment variables the robot software reads at runtime (telemetry, robot ID, camera orientation). |
| `HARDWARE.md` | Docs | Bill of materials — the physical components needed to build the robot. |
| `WIRING.md` | Docs | Pin-by-pin wiring map between the Raspberry Pi and each hardware component. |

## `install_robot.sh`

The single entry point a user runs to set up everything. It:

1. `chmod +x` on `setup.sh` and `verify_hardware.py`.
2. **Phase 1** — runs `./setup.sh` to install dependencies and configure the system.
3. **Phase 2** — runs `python3 verify_hardware.py` as a diagnostic (allowed to fail, since interfaces enabled in Phase 1 need a reboot to activate).
4. Prints a completion banner instructing the user to `sudo reboot`.

## `setup.sh`

Does the real provisioning work, in order:

1. **System update** — `apt-get update && apt-get upgrade -y`.
2. **Install packages** — Python venv/pip/dev headers, `i2c-tools`, `libgpiod`, camera tooling (`v4l-utils`, `libcamera-dev`/`apps`), `pigpio`, audio dev libs (`portaudio19-dev`, `libasound2-dev`), build tools, `libturbojpeg0-dev`.
3. **Enable hardware interfaces** via `raspi-config nonint`: I2C, SPI, hardware serial, camera.
4. **Enable and start `pigpiod`** (daemon required for PWM/GPIO control).
5. **Apply boot config** — appends `config.txt.additions` to `/boot/firmware/config.txt` if not already present (guarded by a `grep` check so it's idempotent).
6. **Install `robot.service`** — substitutes the current `$USER` in place of the hardcoded `yahav` user, copies it to `/etc/systemd/system/`, reloads systemd, and enables it to start on boot.
7. **Set permissions** — adds the current user to the `i2c`, `video`, `gpio`, and `dialout` groups (needed for hardware access without root).

A reboot is required afterward for the boot config changes, interface enables, and group membership to take effect.

## `verify_hardware.py`

Standalone diagnostic script (also invoked by `install_robot.sh`). Checks, printing PASS/FAIL for each:

- **Python packages**: `cv2` (OpenCV), `smbus2` (I2C), `sounddevice` (audio).
- **System interfaces**: `/dev/i2c-1` (I2C bus), `/dev/ttyACM0` or `/dev/ttyUSB0` (Arduino/Lidar serial), `/sys/class/pwm/pwmchip0` (hardware PWM), `/dev/gpiochip0`/`/dev/gpiochip4` (GPIO, RP1 chip on Pi 5).
- **I2C scan**: probes addresses `0x68`/`0x69` for the ICM20948 IMU.
- **Camera**: runs `libcamera-hello --list-cameras` and checks for a detected camera.
- **Audio**: runs `arecord -l` and checks for a capture soundcard (the I2S mic).

Exits non-zero if any critical check fails (serial is informational-only since it depends on external hardware being plugged in).

## `config.txt.additions`

Snippet of Raspberry Pi boot firmware config appended to `/boot/firmware/config.txt`. Key settings:

- `dtparam=i2c_arm=on`, `dtparam=i2s=on` — enable I2C (IMU) and I2S (microphone) buses.
- `dtparam=audio=on` — loads the onboard audio driver.
- `camera_auto_detect=1` / `display_auto_detect=1` — auto-load overlays for connected camera/display.
- `dtoverlay=vc4-kms-v3d`, `max_framebuffers=2` — GPU/display driver.
- `arm_64bit=1`, `arm_boost=1` — 64-bit mode, max clock/performance.
- `[cm4]` / `[cm5]` sections — USB host-mode overlays specific to Compute Module 4/5 variants (not relevant to a standard Pi 5, included for compatibility).

## `robot.service`

systemd unit that runs the robot autonomously on boot:

- Starts after `network.target` and `pigpiod.service` (depends on `pigpiod` being up for GPIO/PWM).
- Runs as the installing user, working directory `AutonomousWarfare/AutonomousWarfare`.
- `ExecStart` runs `./start_system.sh`, which launches the camera pipeline and the docker-compose stack.
- `Restart=always` — automatically restarts the robot software if it crashes.
- Logs to the systemd journal (`journalctl -u robot.service`).

`setup.sh` templates the `User=` and working-directory paths to the current `$USER` before installing this unit.

## `.env.example`

Template for a `.env` file consumed by the robot software at runtime (copy to `.env` and fill in real values — not consumed by the setup scripts themselves):

- `ROBOT_ID` — identifies this robot instance to telemetry and the tactical brain.
- `INFLUX_URL` / `INFLUX_TOKEN` / `INFLUX_ORG` / `INFLUX_BUCKET` — InfluxDB connection for telemetry logging.
- `CAMERA_INVERTED` — set `false` if the camera is mounted upright rather than inverted.
- `ROS_DOMAIN_ID` (optional, commented out) — isolates ROS2 traffic when running multiple robots/networks.

## `HARDWARE.md`

Bill of materials: the physical parts list to build the robot (Raspberry Pi 5, L298N motor driver, DC motors, YDLidar, ICM20948 IMU, Arduino, steering/flag servos, Pi Camera Module 3, ICS-43434 I2S mic, LiPo battery, jumper wires), with approximate quantities and prices.

## `WIRING.md`

Pin-level wiring reference mapping each component to specific Raspberry Pi physical pins / GPIO / BCM numbers:

- **L298N motor driver** — PWM and direction pins for left/right motors, shared ground.
- **ICM20948 IMU** — I2C1 (SDA/SCL) plus power.
- **Arduino** — connected via USB serial (`/dev/ttyACM0`) for steering/flag control.
- **YDLidar** — connected via USB serial (`/dev/ttyUSB0`).
- **Pi Camera** — MIPI CSI ribbon cable.
- **ICS-43434 I2S mic** — BCLK/WS/SD pins plus channel-select-to-ground wiring.

Also includes power-supply warnings (never power motors from the Pi's 5V rail — use a dedicated battery with shared ground) and an ASCII block diagram of how everything connects.

## Typical Setup Flow

1. Wire the hardware per `WIRING.md`, using `HARDWARE.md` as the parts list.
2. Copy `.env.example` to `.env` and fill in real values.
3. Run `./install_robot.sh` (which runs `setup.sh` then `verify_hardware.py`).
4. `sudo reboot`.
5. After reboot, `robot.service` starts automatically; optionally re-run `python3 verify_hardware.py` to confirm all interfaces are now active.
