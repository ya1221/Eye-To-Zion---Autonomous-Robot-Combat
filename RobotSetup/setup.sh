#!/bin/bash
# ==============================================================================
# Eye-To-Zion - Setup Script for Raspberry Pi 5 (Bookworm 64-bit)
# This script installs all system dependencies, configures hardware interfaces,
# sets up a Python virtual environment, and prepares the system for the robot.
# ==============================================================================
set -e

echo "Starting Eye-To-Zion Setup..."

# 1. System Update
echo "--> Updating system packages..."
sudo apt-get update && sudo apt-get upgrade -y

# 2. Install System Dependencies
echo "--> Installing required system packages..."
sudo apt-get install -y \
    python3-venv python3-pip python3-dev \
    i2c-tools libgpiod-dev gpiod \
    v4l-utils libcamera-dev libcamera-apps \
    pigpio python3-pigpio \
    portaudio19-dev libasound2-dev \
    cmake build-essential \
    libturbojpeg0-dev

# 3. Enable Hardware Interfaces
echo "--> Configuring hardware interfaces via raspi-config..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_camera 0



# 5. Enable and start pigpiod
echo "--> Enabling and starting pigpiod service..."
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# 6. Apply Boot Configurations
echo "--> Applying boot configurations to /boot/firmware/config.txt..."
if ! grep -q "dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4" /boot/firmware/config.txt; then
    echo "Appending custom config to /boot/firmware/config.txt"
    cat config.txt.additions | sudo tee -a /boot/firmware/config.txt > /dev/null
fi

# 7. Setup Systemd Service (Optional, uncomment if you want to install it)
echo "--> Configuring and installing robot.service..."
sed -i "s/yahav/$USER/g" robot.service
sudo cp robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot.service

# 8. Set Permissions
echo "--> Setting permissions..."
sudo usermod -a -G i2c,video,gpio,dialout $USER

echo "=============================================================================="
echo "Setup Complete!"
echo "A REBOOT IS HIGHLY RECOMMENDED to apply boot configurations and permissions."
echo "Run 'sudo reboot' to finish."
echo "=============================================================================="