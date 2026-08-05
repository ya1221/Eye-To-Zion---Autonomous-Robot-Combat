#!/usr/bin/env python3
"""
Quick sanity-check: send test commands to the Arduino over /dev/ttyUSB0
and print any response it sends back.

Protocol (from motor_driver.cpp):
  S<left_rad>,<right_rad>\n   — steering
  F1\n / F0\n                  — shooting on/off
"""

import serial
import time
import sys

PORT = "/dev/ttyUSB0"
BAUD = 115200
TIMEOUT = 2  # seconds

def main():
    print(f"[*] Opening {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"[!] FAILED to open {PORT}: {e}")
        sys.exit(1)

    print(f"[+] Port opened. Waiting 2.5s for Arduino bootloader reset...")
    time.sleep(2.5)

    # Flush any bootloader garbage
    ser.reset_input_buffer()

    # --- Test 1: Send steering command ---
    cmd = "S0.0000,0.0000\n"
    print(f"\n[>] Sending steering:  {cmd.strip()}")
    ser.write(cmd.encode())
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting or 1)
    if resp:
        print(f"[<] Arduino replied:   {resp.decode(errors='replace').strip()}")
    else:
        print("[<] (no response — Arduino may not echo steering commands)")

    # --- Test 2: Send fire ON ---
    cmd = "F1\n"
    print(f"\n[>] Sending fire ON:   {cmd.strip()}")
    ser.write(cmd.encode())
    time.sleep(0.5)
    resp = ser.read(ser.in_waiting or 1)
    if resp:
        print(f"[<] Arduino replied:   {resp.decode(errors='replace').strip()}")
    else:
        print("[<] (no response)")

    # --- Test 3: Send fire OFF ---
    cmd = "F0\n"
    print(f"\n[>] Sending fire OFF:  {cmd.strip()}")
    ser.write(cmd.encode())
    time.sleep(0.5)
    resp = ser.read(ser.in_waiting or 1)
    if resp:
        print(f"[<] Arduino replied:   {resp.decode(errors='replace').strip()}")
    else:
        print("[<] (no response)")

    # --- Drain anything else ---
    time.sleep(0.5)
    leftover = ser.read(ser.in_waiting or 1)
    if leftover:
        print(f"\n[<] Remaining data:    {leftover.decode(errors='replace').strip()}")

    ser.close()
    print("\n[+] Test complete. Port closed.")

if __name__ == "__main__":
    main()
