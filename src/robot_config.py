
import vex
from stddefs import *

global brain
brain = Brain()
global master
master = vex.Controller()

# Right Drive
# Back Right Motor looking towards the front
global drive_r1
drive_r1 = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO_6_1, True)
# Front Right Motor looking towards the front
global drive_r2
drive_r2 = vex.Motor(vex.Ports.PORT3, vex.GearSetting.RATIO_6_1, True)
# Right Drive Motor Group
global drive_r
drive_r = vex.MotorGroup(drive_r1, drive_r2)

# Left Drive
# Back Left Motor looking towards the front
global drive_l1
drive_l1 = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO_6_1, False) 
# Front Left Motor looking towards the front
global drive_l2
drive_l2 = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO_6_1, False)
# Left Drive Motor Group
global drive_l
drive_l = vex.MotorGroup(drive_l1, drive_l2)

# Drivetrain
# Not used in driving; Only for auton
drive1 = DriveTrain(drive_l, drive_r, 311.15, 260.34, 0.75)


# Intake System
# Left Intake looking towards the front
global intake1
intake1 = vex.Motor(vex.Ports.PORT7, vex.GearSetting.RATIO_18_1, True)
# Right Intake looking towards the front
global intake2
intake2 = vex.Motor(vex.Ports.PORT8, vex.GearSetting.RATIO_18_1, False)
# Top Outtake 
global outtake1
outtake1 = vex.Motor(vex.Ports.PORT5, vex.GearSetting.RATIO_6_1, True)
# Intake Motor Group
global intake
intake = vex.MotorGroup(intake1, intake2, outtake1)
intake.set_velocity(300, RPM)
outtake1.set_velocity(600, RPM)
# Bottom Outtake
global outtake2
outtake2 = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO_18_1, False)
outtake2.set_velocity(400, RPM)


# Pneumatic System
descorer = DigitalOut(brain.three_wire_port.a)
unloader = DigitalOut(brain.three_wire_port.b)
splitter = DigitalOut(brain.three_wire_port.c)
# pneumenoultramicroscopicsillicavolcaniconiosis 
# Cylinders
#global wing_r
#wing_r = vex.DigitalOut(brain.three_wire_port.a)
#global wing_l
#wing_l = vex.DigitalOut(brain.three_wire_port.b)
#global intake_fold
#intake_fold = vex.DigitalOut(brain.three_wire_port.c)

# Sensors
global imu
imu = vex.Inertial(vex.Ports.PORT15)

optical1 = Optical(Ports.PORT11)
optical2 = Optical(Ports.PORT14)

# Rotation Sensors
turnr = Rotation(Ports.PORT12)
linearr= Rotation(Ports.PORT13)

#global clock
#clock = vex.Timer()
#  global auton_selector
# auton_selector = vex.DigitalIn(brain.three_wire_port.h)

# vision__RED_BOX = Signature(1, 14779, 15299, 15039,685, 1137, 911,2.5, 0)
# vision__BLUE_BOX = Signature(2, -2397, -2067, -2232,7599, 8117, 7858,2.5, 0)
# vision__RED2 = Signature(3, 6085, 9305, 7695,-2073, -763, -1418,2.5, 0)
# vision__RED3 = Signature(4, 6517, 7831, 7174,-1867, -353, -1110,2.5, 0)
# vision__RED4 = Signature(5, 7439, 9649, 8544,-1747, -183, -965,2.5, 0)
# vision__RED5 = Signature(6, 5335, 8245, 6790,-1365, 309, -528,2.5, 0)
# #vision = Vision(Ports.PORT11, 50, vision__RED_BOX, vision__BLUE_BOX)
# vision = Vision(Ports.PORT11, 50, vision__RED_BOX, vision__BLUE_BOX, vision__RED2, vision__RED3, vision__RED4, vision__RED5)


# Globals
global all_globals
all_globals = TrackedGlobals(0, 10.75, (3600 / 3593.6))


