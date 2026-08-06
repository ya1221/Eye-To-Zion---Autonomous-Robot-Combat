#!/bin/bash
set -e

echo "================================================================="
echo "             Eye-To-Zion Master Installation Script              "
echo "================================================================="

# Step 1: Make scripts executable
chmod +x setup.sh
chmod +x verify_hardware.py

# Step 2: Run the main system setup
# This handles apt packages, pip packages, pigpiod, and copies config.txt.additions
# It also configures and starts the robot.service systemd daemon.
echo -e "\n\n>>> PHASE 1: Running System Setup (Dependencies & Configuration)..."
./setup.sh

# Step 3: Run the hardware verification
# Note: Because the I2C, Camera, and PWM interfaces were just turned on in Phase 1,
# some of these tests might fail until the Raspberry Pi actually reboots to load them.
echo -e "\n\n>>> PHASE 2: Running Pre-Reboot Hardware Diagnostic..."
python3 verify_hardware.py || true

# Step 4: Final Instructions
echo -e "\n\n================================================================="
echo "                     INSTALLATION COMPLETE!                      "
echo "================================================================="
echo "The setup script has successfully configured:"
echo "  ✓ System Dependencies"
echo "  ✓ Hardware Overlays (config.txt)"
echo "  ✓ Background Service (robot.service)"
echo ""
echo "CRITICAL NEXT STEP:"
echo "Your Raspberry Pi must be rebooted to activate the I2C, Camera, and PWM hardware."
echo "Once rebooted, the robot will start automatically in the background."
echo ""
echo "Please run this command now:"
echo "    sudo reboot"
echo "================================================================="
