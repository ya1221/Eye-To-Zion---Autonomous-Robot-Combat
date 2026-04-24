from gpiozero import Robot
from gpiozero.pins.lgpio import LGPIOFactory
import time

# 1. Initialize the LGPIO factory for Raspberry Pi 5 compatibility [cite: 112]
# The 'chip=0' parameter usually refers to the user-facing GPIO header [cite: 112, 113]
factory = LGPIOFactory(chip=0)

# 2. Define your robot's pins [cite: 107, 112]
# Format: Robot(left=(forward_pin, backward_pin), right=(forward_pin, backward_pin))
# Replace these numbers with the GPIO pins connected to your motor driver.
robot = Robot(left=(17, 27), right=(22, 23), pin_factory=factory)

print("Moving forward...")
robot.forward(1)
time.sleep(1)

# print("Turning right...")
# robot.right()
# time.sleep(1)

print("Stopping...")
robot.stop()