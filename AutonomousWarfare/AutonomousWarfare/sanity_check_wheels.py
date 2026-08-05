from gpiozero import Robot, AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
import time

factory = LGPIOFactory(chip=0)

robot = Robot(left=(17, 27), right=(22, 23), pin_factory=factory)

# Servo — replace 18 with whichever GPIO the servo's signal wire is on
servo = AngularServo(18, min_angle=-90, max_angle=90, pin_factory=factory)

try:
    print("Moving forward...")
    # robot.forward(1)
    # time.sleep(1)
    # robot.stop()

    # print("Moving servo to 45°...")
    # servo.angle = 45
    # robot.forward(1)
    # time.sleep(3)

    print("Moving servo to -45°...")
    servo.angle = 0.0
    robot.forward(0.3)
    time.sleep(15)

    # print("Centering servo...")
    # servo.angle = 0
    # robot.forward(1)
    # time.sleep(2)

finally:
    robot.close()
    servo.close()