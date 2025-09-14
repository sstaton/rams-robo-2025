
import vex
from stddefs import *

global brain
brain = Brain()
global master
master = vex.Controller()

# Right Drive
global drive_r1
drive_r1 = vex.Motor(vex.Ports.PORT9, vex.GearSetting.RATIO_6_1, True)
global drive_r2
drive_r2 = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO_6_1, True)

global drive_r
drive_r = vex.MotorGroup(drive_r1, drive_r2)

# Left Drive
global drive_l1
drive_l1 = vex.Motor(vex.Ports.PORT10, vex.GearSetting.RATIO_6_1, False) 
global drive_l2
drive_l2 = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO_6_1, False)

global drive_l
drive_l = vex.MotorGroup(drive_l1, drive_l2)


# Subsystem 3
global intake1
intake1 = vex.Motor(vex.Ports.PORT8, vex.GearSetting.RATIO_18_1, True)
global intake2
intake2 = vex.Motor(vex.Ports.PORT3, vex.GearSetting.RATIO_18_1, False)

global intake
intake = vex.MotorGroup(intake1, intake2)
intake.set_velocity(300, RPM)
# Cylinders
#global wing_r
#wing_r = vex.DigitalOut(brain.three_wire_port.a)
#global wing_l
#wing_l = vex.DigitalOut(brain.three_wire_port.b)
#global intake_fold
#intake_fold = vex.DigitalOut(brain.three_wire_port.c)

# Sensors
global imu
imu = vex.Inertial(vex.Ports.PORT21)
#global clock
#clock = vex.Timer()
# global auton_selector
# auton_selector = vex.DigitalIn(brain.three_wire_port.h)

# Globals
global all_globals
all_globals = TrackedGlobals(0, 10.75, (3600 / 3593.6))
