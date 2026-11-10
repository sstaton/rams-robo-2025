
from stddefs import *
from vex import *
global brain
brain = vex.Brain()
global master
master = vex.Controller()
global drive_r2
# Right Drive
global drive_r1
drive_r1 = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO_6_1, False)
drive_r2 = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO_6_1, False)

# drive_r2 = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO_6_1, False)

global drive_r
drive_r = vex.MotorGroup(drive_r1, drive_r2)
global drive_l2
# Left Drive
global drive_l1
drive_l1 = vex.Motor(vex.Ports.PORT5, vex.GearSetting.RATIO_6_1, True)
drive_l2 = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO_6_1, True)


global drive_l
drive_l = vex.MotorGroup(drive_l1, drive_l2)

global arm
arm1 = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
arm2 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
arm = MotorGroup(arm1, arm2)
# # Subsystem 3
# global intake
# intake = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO_18_1, False)
# # global hang
# # hang = vex.Motor(vex.Ports.PORT7, vex.GearSetting.RATIO_36_1, False)

# # Cylindersik = 8/
# global wing_r
# wing_r = vex.DigitalOut(brain.three_wire_port.a)
# global wing_l
# wing_l = vex.DigitalOut(brain.three_wire_port.b)
# global intake_fold
# intake_fold = vex.DigitalOut(brain.three_wire_port.c)

# # Sensors
global imu
imu = vex.Inertial(Ports.PORT20)
# global clock
# clock = vex.Timer()
# # global auton_selector
# # auton_selector = vex.DigitalIn(brain.three_wire_port.h)

# Globals
global all_globals
all_globals = TrackedGlobals(0, 10.75, (3600 / 3593.6))
# Initialize motors on ports 1 and 10
# global left_motor
# global right_motor
# left_motor = vex.Motor(vex.Ports.PORT1)
# right_motor = vex.Motor(vex.Ports.PORT10)

# global drivetrain
# # Create a drivetrain object
# drivetrain = vex.DriveTrain(left_motor, right_motor)

# # Drive forward for 500 mm
# drivetrain.drive_for("FORWARD", 500, MM)

# # Turn right 90 degrees
# drivetrain.turn_for("RIGHT", 90, "degrees")

# # Stop the drivetrain
# drivetrain.stop()
# # from simple_drivetrain import Drivetrain

# # Define drivetrain with motors on ports 1 and 10
# # drivetrain = DriveTrain(left_motor_port=1, right_motor_port=10)

# # Drive forward at 50% speed for 2 seconds
# vex.drivetrain.drive(speed=50, duration=2)

# # Rotate in place to the right
# drivetrain.turn(direction="right", speed=30, duration=1)

# # Stop the drivetrain
# drivetrain.stop()
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