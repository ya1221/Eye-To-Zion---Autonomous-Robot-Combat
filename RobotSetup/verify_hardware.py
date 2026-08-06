#!/usr/bin/env python3
import os
import sys
import subprocess

def print_result(name, passed, details=""):
    status = "\033[92mPASS\033[0m" if passed else "\033[91mFAIL\033[0m"
    print(f"[{status}] {name: <30} {details}")
    return passed

def check_package(pkg_name):
    try:
        __import__(pkg_name)
        return True
    except ImportError:
        return False

def main():
    print("========================================")
    print("Eye-To-Zion Hardware Diagnostic Tool")
    print("========================================")
    all_passed = True

    # 1. Check Python Packages
    print("\n--- Checking Critical Python Dependencies ---")
    pkgs = {'cv2': 'OpenCV', 'smbus2': 'SMBus (I2C)', 'sounddevice': 'Audio'}
    for mod, desc in pkgs.items():
        if not print_result(f"Package: {desc}", check_package(mod)):
            all_passed = False

    # 2. Check System Interfaces
    print("\n--- Checking System Interfaces ---")
    
    # I2C
    i2c_ok = os.path.exists('/dev/i2c-1')
    if not print_result("I2C Bus (/dev/i2c-1)", i2c_ok):
        all_passed = False

    # Serial (Arduino / Lidar)
    arduino_ok = os.path.exists('/dev/ttyACM0') or os.path.exists('/dev/ttyUSB0')
    print_result("Serial (Arduino/Lidar)", arduino_ok, "(Requires hardware connected via USB)")

    # PWM
    pwm_ok = os.path.exists('/sys/class/pwm/pwmchip0')
    if not print_result("Hardware PWM (/sys/class/pwm)", pwm_ok):
        all_passed = False
        
    # GPIO Chip (RP1 on Pi 5)
    gpiod_ok = os.path.exists('/dev/gpiochip0') or os.path.exists('/dev/gpiochip4')
    if not print_result("GPIO Chip (libgpiod)", gpiod_ok):
        all_passed = False

    # 3. Scan I2C Bus for IMU
    print("\n--- Scanning I2C Bus for Devices ---")
    if i2c_ok:
        try:
            import smbus2
            bus = smbus2.SMBus(1)
            found = False
            for addr in [0x68, 0x69]:
                try:
                    bus.read_byte(addr)
                    print_result(f"ICM20948 IMU (0x{addr:02X})", True)
                    found = True
                    break
                except Exception:
                    pass
            if not found:
                print_result("ICM20948 IMU", False, "(Expected at 0x68 or 0x69)")
                all_passed = False
        except Exception as e:
            print_result("I2C Scan", False, str(e))
            all_passed = False
    else:
        print_result("I2C Scan", False, "I2C bus not available")

    # 4. Check Camera
    print("\n--- Checking Camera ---")
    try:
        res = subprocess.run(['libcamera-hello', '--list-cameras'], capture_output=True, text=True)
        cam_ok = "Available cameras" in res.stdout and "0 cameras" not in res.stdout
        print_result("Pi Camera Module", cam_ok)
    except FileNotFoundError:
        print_result("Pi Camera Module", False, "libcamera-apps not installed")
        all_passed = False

    # 5. Check Audio
    print("\n--- Checking Audio ---")
    try:
        res = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
        mic_ok = "card" in res.stdout.lower() and "no soundcards found" not in res.stdout.lower()
        if not print_result("ICS-43434 I2S Mic (ALSA)", mic_ok, "" if mic_ok else "(No capture soundcard found)"):
            all_passed = False
    except FileNotFoundError:
        if not print_result("ICS-43434 I2S Mic (ALSA)", False, "(alsa-utils 'arecord' not installed)"):
            all_passed = False

    print("\n========================================")
    if all_passed:
        print("\033[92mALL CRITICAL CHECKS PASSED!\033[0m")
        print("Your system is ready for Eye-To-Zion.")
    else:
        print("\033[91mSOME CHECKS FAILED.\033[0m")
        print("Please review the errors above and ensure all hardware is connected")
        print("and the setup.sh script ran successfully.")
        sys.exit(1)

if __name__ == "__main__":
    main()
