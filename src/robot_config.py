# ./.
# global brain
brain = Brain()
# global master
master = Controller()

# Right Drive
# global drive_r1

drive_r1 = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)

drive_r2 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

# global drive_r
drive_r = MotorGroup(drive_r1, drive_r2)
# Left Drive
# global drive_l1
drive_l1 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
# global drive_l2
drive_l2 = Motor(Ports.PORT9F, GearSetting.RATIO_18_1, False)

# global drive_l
# drivetrain = MotorGroup(drive_r1, drive_r2, drive_l1, drive_l2)

# Subsystem 3
# global intake
intake_1 = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
# global hang
intake_2 = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

intake_3 = Motor(Ports.PORT13, Gearsetting.RATIO_6_1, False)

intake_4 = Motor(Ports.PORT12, Gearsetting.RATIO_6_1, False)

intake_5 = Motor(Ports.PORT3, Gearsetting.RATIO_18_1, True)

intakegroup = MotorGroup(intake_1, intake_2, intake_3)

intakegroup_2 = MotorGroup(intake_4, intake_5)

drive_1.set_turn_velocity(75, PERCENT)

# Cylinders
# global wing_r
wing_r = DigitalOut(brain.three_wire_port.a)
# global wing_l
wing_l = DigitalOut(brain.three_wire_port.b)
# global intake_fold
intake_fold = DigitalOut(brain.three_wire_port.c)

# Sensors
# global imu
imu = Inertial(Ports.PORT17)
# global clock
clock = Timer()
# global auton_selector
# auton_selector = DigitalIn(brain.three_wire_port.h)

# Globals
# global all_globals
all_globals = TrackedGlobals(0, 10.75, (3600 / 3593.6))