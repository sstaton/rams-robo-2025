from robot_config import *
from movement import * 
from vex import *
from stddefs import *

def autonomous():
    brain.screen.clear_screen()

    # Drive forward for 500 mm
    drive_straight(20, 50, 50)

    # Turn right 90 degrees
    drive_turn(90, 3, 50, 50, True)



    # Drive forward at 50% speed for 2 seconds
    drive_straight(20, 50, 50)

    # Rotate in place to the right
    drive_turn(90, 3, 50, 50, False)

    # # Stop the drivetrain
    # import time

    # class Motor:
    #     def __init__(self, port):
    #         self.port = port

    #     def set_speed(self, speed):
    #         print(f"Motor on port {self.port} set to speed {speed}")

    #     def stop(self):
    #         print(f"Motor on port {self.port} stopped")

    # # Initialize motors
    # left_motor = Motor(1)
    # right_motor = Motor(10)

    # # Drive forward
    # left_motor.set_speed(50)
    # right_motor.set_speed(50)
    # time.sleep(2)  # Drive for 2 seconds

    # # Stop motors
    # left_motor.stop()
    # right_motor.stop()