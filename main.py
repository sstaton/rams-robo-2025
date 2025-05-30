import vex
import math



# ./src/stddefs.py ---
# Transferred from stddefs.h, so incomplete. I've not made shorthand for every
# unit or unit conversion

# Conversions
global RAD_TO_DEG
RAD_TO_DEG = 57.29578

# Wheel sizes
global LARGE_OMNI_DIAM
LARGE_OMNI_DIAM = 4.0
global MEDIUM_OMNI_DIAM
MEDIUM_OMNI_DIAM = 3.25
global SMALL_OMNI_DIAM
SMALL_OMNI_DIAM = 2.75
global LARGE_WHEEL_DIAM
LARGE_WHEEL_DIAM = 5.0
global MEDIUM_WHEEL_DIAM
MEDIUM_WHEEL_DIAM = 4.0
global SMALL_WHEEL_DIAM
SMALL_WHEEL_DIAM = 2.75
global TRACT_WHEEL_DIAM
TRACT_WHEEL_DIAM = 3.25

global LARGE_OMNI_CIRC
LARGE_OMNI_CIRC = LARGE_OMNI_DIAM * math.pi
global MEDIUM_OMNI_CIRC
MEDIUM_OMNI_CIRC = MEDIUM_OMNI_DIAM * math.pi
global SMALL_OMNI_CIRC
SMALL_OMNI_CIRC = SMALL_OMNI_DIAM * math.pi
global LARGE_WHEEL_CIRC
LARGE_WHEEL_CIRC = LARGE_WHEEL_DIAM * math.pi
global MEDIUM_WHEEL_CIRC
MEDIUM_WHEEL_CIRC = MEDIUM_WHEEL_DIAM * math.pi
global SMALL_WHEEL_CIRC
SMALL_WHEEL_CIRC = SMALL_WHEEL_DIAM * math.pi
global TRACT_WHEEL_CIRC
TRACT_WHEEL_CIRC = TRACT_WHEEL_DIAM * math.pi

# Unit shorthand
global REV
REV = vex.RotationUnits.REV
global DEG
DEG = vex.RotationUnits.DEG

# Globally tracked things
class TrackedGlobals:
    def __init__(self, target_heading, wheel_to_wheel_dist, imu_correction):
        self.target_heading = target_heading
        self.wheel_to_wheel_dist = wheel_to_wheel_dist
        self.imu_correction = imu_correction
    def set_target_heading(self, value):
        self.target_heading = value
    def inc_target_heading(self, value):
        self.target_heading += value



# ./src/robot_config.py ---

global brain
brain = vex.Brain()
global master
master = vex.Controller()

# Right Drive
global drive_r1
drive_r1 = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO_6_1, False)
global drive_r2
drive_r2 = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO_6_1, False)
global drive_r3
drive_r3 = vex.Motor(vex.Ports.PORT3, vex.GearSetting.RATIO_6_1, False)
global drive_r4
drive_r4 = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO_6_1, False)

global drive_r
drive_r = vex.MotorGroup(drive_r1, drive_r2, drive_r3, drive_r4);

# Left Drive
global drive_l1
drive_l1 = vex.Motor(vex.Ports.PORT5, vex.GearSetting.RATIO_6_1, True)
global drive_l2
drive_l2 = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO_6_1, True)
global drive_l3
drive_l3 = vex.Motor(vex.Ports.PORT7, vex.GearSetting.RATIO_6_1, True)
global drive_l4
drive_l4 = vex.Motor(vex.Ports.PORT8, vex.GearSetting.RATIO_6_1, True)

global drive_l
drive_l = vex.MotorGroup(drive_l1, drive_l2, drive_l3, drive_l4);

# Subsystem 3
global intake
intake = vex.Motor(vex.Ports.PORT9, vex.GearSetting.RATIO_18_1, False);
global hang
hang = vex.Motor(vex.Ports.PORT10, vex.GearSetting.RATIO_36_1, False);

# Cylinders
global wing_r
wing_r = vex.DigitalOut(brain.three_wire_port.a)
global wing_l
wing_l = vex.DigitalOut(brain.three_wire_port.b)
global intake_fold
intake_fold = vex.DigitalOut(brain.three_wire_port.c)

# Sensors
global imu
imu = vex.Inertial(vex.Ports.PORT11)
global clock
clock = vex.Timer()
global auton_selector
auton_selector = vex.DigitalIn(brain.three_wire_port.h)

# Globals
global all_globals
all_globals = TrackedGlobals(0, 10.75, (3600 / 3593.6))



# ./src/util.py ---

# Top of file is new functions
# Bottom of file is making shorthand for long function names, like:
# src.robot_config.drive_r.position(vex.RotationUnits.REV)

# UTILITY FUNCTIONS

class EdgeDetection:
    def __init__(self, value):
        self.prev_val = value

    # Mostly used to toggle things on button press
    # Rising edge
    def is_redge(self, value):
        edge = False
        if self.prev_val < value:
            edge = True

        self.prev_val = value
        return edge

    # Mostly used to toggle things on button release
    # Falling edge
    def is_fedge(self, value):
        edge = False
        if self.prev_val > value:
            edge = True

        self.prev_val = value
        return edge

    # Not sure what the use case is, but I added it before I realized that
    def is_edge(self, value):
        edge = False
        if not self.prev_val == value:
            edge = True

        self.prev_val = value
        return edge

def stop_distance(velocity, acceleration, target_velocity = 0):
    return -(target_velocity ** 2 - velocity ** 2 ) / (2 * acceleration);

def handle_acceleration(position, distance, velocity, max_velocity, acceleration, tick_rate, do_decel):
    if (abs(position) + stop_distance(velocity, acceleration) >= abs(distance)) and do_decel:
        return velocity - acceleration / tick_rate
    if velocity < max_velocity:
        return velocity + acceleration * tick_rate
    return max_velocity

def within_range(value, base_value, range):
    if value <= base_value + range and value >= base_value - range:
        return True
    return False

# SHORTHAND
global DRIVE_REV_TO_IN
DRIVE_REV_TO_IN = MEDIUM_OMNI_CIRC * (36.0/48.0)

def pos_drive_r():
    return drive_r.position(REV) * DRIVE_REV_TO_IN
def pos_drive_l():
    return drive_l.position(REV) * DRIVE_REV_TO_IN
def vel_drive_r():
    return drive_r.velocity(vex.RPM) * DRIVE_REV_TO_IN
def vel_drive_l():
    return drive_l.velocity(vex.RPM) * DRIVE_REV_TO_IN

def imu_rotation():
    return imu.rotation() * all_globals.imu_correction

# Controller joystick shorthand
def axis_rx():
    return master.axis1.value()
def axis_ry():
    return master.axis2.value()
def axis_lx():
    return master.axis3.value()
def axis_ly():
    return master.axis4.value()

# Control back button shorthand
def btn_r1():
    return master.buttonR1.pressing()
def btn_r2():
    return master.buttonR2.pressing()
def btn_l1():
    return master.buttonL1.pressing()
def btn_l2():
    return master.buttonL2.pressing()

# Controller front button shorthand
def btn_a():
    return master.buttonA.pressing()
def btn_b():
    return master.buttonB.pressing()
def btn_x():
    return master.buttonX.pressing()
def btn_y():
    return master.buttonY.pressing()

def btn_right():
    return master.buttonRight.pressing()
def btn_left():
    return master.buttonLeft.pressing()
def btn_up():
    return master.buttonUp.pressing()
def btn_down():
    return master.buttonDown.pressing()



# ./src/pid.py ---

class Pid:
    def __init__(self, init_kp, init_ki, init_kd):
        self.kp = init_kp
        self.ki = init_ki
        self.kd = init_kd

        self.error = 0
        self.sum = 0
        self.prev_error = 0
        self.deriv = 0

    def tune_kp(self, value, mod):
        self.kp += value * mod
        print("kp: " + self.kp + "")
        brain.screen.set_cursor(3, 4)
        brain.screen.print("kp: " + self.kp + "")

    def tune_ki(self, value, mod):
        self.ki += value * mod
        print("ki: " + self.ki + "")
        brain.screen.set_cursor(3, 4)
        brain.screen.print("ki: " + self.ki + "")

    def tune_kd(self, value, mod):
        self.kd += value * mod
        print("kd: " + self.kd + "")
        brain.screen.set_cursor(3, 4)
        brain.screen.print("kd: " + self.kd + "")

    def adjust(self, setpoint, sensor_value):
        self.error = setpoint - sensor_value
        # Only incrememnt sum if the i loop is used
        if not self.ki == 0:
            self.sum += self.error
        self.deriv = self.error - self.prev_error
        self.prev_error = self.error

        p_term = self.error * self.kp
        i_term = self.sum * self.ki
        d_term = self.deriv * self.kd

        return p_term + i_term + d_term



# ./src/movement.py ---

# takes inches, target inches per second (velocity),
# inches per second squared (acceleration), and whether to decelerate
def drive_straight(inches, target_ips, ipss, do_decel = True):
    # Loop vex.wait times
    TICK_PER_SEC = 50    # tick per sec
    MSEC_PER_TICK = 20   # ms per tick

    # PID constants
    DRIVE_KP = 0.005
    DRIVE_KI = 0.00
    DRIVE_KD = 1.9

    DIR_KP = 2.925
    DIR_KI = 0.00
    DIR_KD = 0.183

    MULTIPLIER = 1.2        # not sure why I need this, but it makes it so that
    inches *= MULTIPLIER    # passing 4 inches actually moves 4 inches

    drive_r.stop(vex.COAST)
    drive_l.stop(vex.COAST)

    pid_drive_r = Pid(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    pid_drive_l = Pid(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    pid_dir = Pid(DIR_KP, DIR_KI, DIR_KD)

    ips = 0     # current speed in ips
    expected_displacement = 0     # inches travelled since function call
    pos_start_l = drive_l.position(REV)
    pos_start_r = drive_r.position(REV)

    # adjusts velocity for positive/negative distances
    dir_mod = 1 if inches < 0 else -1

    # While speed is positive and we haven't gone further than `inches`
    while ips >= 0 and abs(pos_drive_l() - pos_start_l) < abs(inches):
        # Handle acceleration
        if abs(expected_displacement) + stop_distance(ips, ipss) >= abs(inches) and do_decel:
            ips -= ipss / TICK_PER_SEC           # decel
        elif ips < target_ips:
            ips += target_ips / TICK_PER_SEC     # accel
        else:
            ips = target_ips            # cap vel at target_ips

        # Find expected position
        expected_displacement += ips / TICK_PER_SEC * dir_mod      # dir_mod adjusts for moving fwd/bwd

        # Find actual position
        displacement_l = pos_drive_l() - pos_start_l
        displacement_r = pos_drive_r() - pos_start_r

        adjustment_r = pid_drive_r.adjust(expected_displacement, displacement_r)
        adjustment_l = pid_drive_l.adjust(expected_displacement, displacement_l)
        adjustment_dir = pid_dir.adjust(all_globals.target_heading, displacement_l)

        vel_rpm = ips / DRIVE_REV_TO_IN * 60
        
        drive_r.spin(vex.FORWARD, dir_mod * vel_rpm + adjustment_r - adjustment_dir, vex.RPM)
        drive_l.spin(vex.FORWARD, dir_mod * vel_rpm + adjustment_l + adjustment_dir, vex.RPM)

        vex.wait(MSEC_PER_TICK, vex.MSEC)
        
    if do_decel:
        drive_r.stop(vex.BRAKE)
        drive_l.stop(vex.BRAKE)
    else:
        drive_r.stop(vex.COAST)
        drive_l.stop(vex.COAST)

def drive_turn(degrees, outer_radius, target_ips, ipss, reversed):
    # Loop vex.wait times
    TICK_PER_SEC = 50
    MSEC_PER_TICK = 20

    # PID constants
    DRIVE_KP = 0.005   
    DRIVE_KI = 0.00   
    DRIVE_KD = 1.9

    # Update robot's target heading
    all_globals.inc_target_heading(degrees)

    pid_drive_r = Pid(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    pid_drive_l = Pid(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    
    ips = 0
    outer_displacement = 0
    pos_start_r = pos_drive_r()
    pos_start_l = pos_drive_l()

    # radius of turn
    inner_radius = outer_radius - all_globals.wheel_to_wheel_dist
    radius_ratio = inner_radius / outer_radius

    dir_mod = 1 if degrees > 0 else -1

    while ips >= 0:
        # Find distance travelled since function call
        displacement_r = pos_drive_r() - pos_start_r
        displacement_l = pos_drive_l() - pos_start_l

        # Degrees remaining to complete turn
        degrees_remaining = all_globals.target_heading - imu_rotation()

        # Handle acceleration
        # radians remaining * inches per radian; in other words, inches remaining
        if abs(degrees_remaining / RAD_TO_DEG * outer_radius) - stop_distance(ips, ipss) <= 0:
            ips -= ipss / TICK_PER_SEC      # decel
        elif ips < target_ips:
            ips += ipss / TICK_PER_SEC      # accel
        else:
            ips = target_ips                # cap at target_ips

        # Convert ips to rpm
        outer_vel_rpm = ips / DRIVE_REV_TO_IN * 60 * (-1 if reversed else 1)
        inner_vel_rpm = outer_vel_rpm * radius_ratio * (-1 if reversed else 1)

        outer_displacement += ips / TICK_PER_SEC
        inner_displacement = outer_displacement * radius_ratio

        # Get PID adjustments; move the drive
        if (reversed and degrees > 0) or ((not reversed) and (not degrees > 0)):        # left is inner side
            adjustment_r = pid_drive_r.adjust(outer_displacement, displacement_r)
            adjustment_l = -1 * pid_drive_l.adjust(inner_displacement, displacement_l)

            drive_r.spin(vex.FORWARD, outer_vel_rpm + adjustment_r, vex.RPM)
            drive_l.spin(vex.FORWARD, inner_vel_rpm + adjustment_l, vex.RPM)
        else:                                                                           # right is inner side
            adjustment_r = -1 * pid_drive_r.adjust(inner_displacement, displacement_r)
            adjustment_l = pid_drive_l.adjust(outer_displacement, displacement_l)

            drive_r.spin(vex.FORWARD, outer_vel_rpm + adjustment_r, vex.RPM)
            drive_l.spin(vex.FORWARD, inner_vel_rpm + adjustment_l, vex.RPM)

        # Exit loop if we're past the desired angle
        if degrees_remaining * dir_mod < 0:
            break

        vex.wait(MSEC_PER_TICK, vex.MSEC)
    drive_r.stop(vex.BRAKE)
    drive_l.stop(vex.BRAKE)

# slightly better drive_straight, in theory
# broken right now
def drive_linear(inches, max_ips, ipss, do_decel = True):
    # Loop vex.wait times
    TICKS_PER_SEC = 50
    MSEC_PER_TICK = 20

    # PID constants
    DRIVE_KP = 0.005
    DRIVE_KI = 0
    DRIVE_KD = 1.9

    pid_drive_r = Pid(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    pid_drive_l = Pid(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    pid_dir = Pid(DRIVE_KP, DRIVE_KI, DRIVE_KD)     # probably should have its own kP, kI, and kD values

    # Start positions
    start_pos_r = pos_drive_r()
    start_pos_l = pos_drive_l()
    displacement_r = pos_drive_r() - start_pos_r
    displacement_l = pos_drive_l() - start_pos_l
    avg_displacement = (displacement_l + displacement_r) / 2.0

    # Setting initial velocities to allow for function chains
    target_vel_r = vel_drive_r()
    target_vel_l = vel_drive_l()

    # adjusts velocity for postive/negative distances
    dir_mod = 1 if inches > 0 else -1

    while abs(avg_displacement) < abs(inches):
        # Update distance travelled since function call
        displacement_r = pos_drive_r() - start_pos_r
        displacement_l = pos_drive_l() - start_pos_l
        avg_displacement = (displacement_r + displacement_l) / 2.0

        # Handle acceleration
        target_vel_r = handle_acceleration(displacement_r, inches, target_vel_r, max_ips, ipss, TICKS_PER_SEC, do_decel)
        target_vel_l = handle_acceleration(displacement_l, inches, target_vel_l, max_ips, ipss, TICKS_PER_SEC, do_decel)

        # Get PID adjustments
        adjustment_r = pid_drive_r.adjust(target_vel_r, vel_drive_r())
        adjustment_l = pid_drive_l.adjust(target_vel_l, vel_drive_l())
        adjustment_dir = pid_dir.adjust(all_globals.target_heading, imu_rotation())

        # Convert ips to rpm
        r_vel_rpm = target_vel_r / DRIVE_REV_TO_IN  * 60
        l_vel_rpm = target_vel_l / DRIVE_REV_TO_IN  * 60

        # Move drive
        drive_r.spin(vex.FORWARD, dir_mod * r_vel_rpm + adjustment_r + adjustment_dir, vex.RPM)
        drive_l.spin(vex.FORWARD, dir_mod * l_vel_rpm + adjustment_l + adjustment_dir, vex.RPM)

        vex.wait(MSEC_PER_TICK, vex.MSEC)

    if do_decel:
        drive_r.stop(vex.BRAKE)
        drive_l.stop(vex.BRAKE)
    else:
        drive_r.stop(vex.COAST)
        drive_l.stop(vex.COAST)

# *Much* faster drive_turn(), but less control over speed, etc.
# Doesn't work well if arcing on full omni drives
# Slightly broken; can only turn in place (if radius_ratio = 1)
def turn_pid(degrees, radius_ratio, direction):
    MSEC_PER_TICK = 20

    TURN_KP = 1.05
    TURN_KI = 0
    TURN_KD = 4.575
    # Update robot's target heading
    all_globals.inc_target_heading(degrees)

    # Create PID object
    drive_pid = Pid(TURN_KP, TURN_KI, TURN_KD)

    # Track the amount of time the robot has been within a certain value
    time_still = 0
    while (time_still < 80):
        if within_range(imu_rotation(), all_globals.target_heading, 3):
            time_still += 20
        else:
            time_still = 0

        # Find new speed of drive
        speed_r = drive_pid.adjust(all_globals.target_heading, imu_rotation()) * direction
        speed_l = speed_r * radius_ratio

        if speed_l > 100 * abs(radius_ratio):
            speed_l = 100 * abs(radius_ratio)
        elif speed_l < -100 * abs(radius_ratio):
            speed_l = -100 * abs(radius_ratio)
        if speed_r > 100 * abs(radius_ratio):
            speed_r = 100 * abs(radius_ratio)
        elif speed_r < -100 * abs(radius_ratio):
            speed_r = -100 * abs(radius_ratio)

        drive_r.spin(vex.FORWARD, speed_l, vex.PERCENT)
        drive_r.spin(vex.FORWARD, speed_l, vex.PERCENT)
        vex.wait(MSEC_PER_TICK, vex.MSEC)

# Same idea as turn_pid(), but for drive_straight()
def straight_pid(dist):
    print("WIP")



# ./src/preauton.py ---

def preauton():
    brain.screen.clear_screen()
    imu.calibrate()

    while imu.is_calibrating():
        vex.wait(20, vex.TimeUnits.MSEC)



# ./src/auton.py ---

def autonomous():
    brain.screen.clear_screen()



# ./src/opcontrol.py ---

# Constants
TNK = 0
TSA = 1
OSA = 2

def opcontrol():
    SENSITIVITY = 0.85

    # Set up edge detection
    fold_switch = EdgeDetection(False)
    wing_r_switch = EdgeDetection(False)
    wing_l_switch = EdgeDetection(False)

    # Reset drive velocity
    drive_l.stop(vex.COAST)
    drive_r.stop(vex.COAST)

    while(True):
        # Drivetrain
        opdrive(TSA, 1.0, SENSITIVITY)

        # Elevation
        hang.spin(vex.FORWARD, (btn_right() - btn_y()) * 100, vex.PERCENT)

        # Set a "shift" key
        shifted = btn_l2()

        # Base layer
        if not shifted:
            # Intake
            intake.spin(vex.FORWARD, (btn_r1() - btn_r2()) * 100, vex.PERCENT)
            # Change intake height
            intake_fold.set(fold_switch.is_redge(btn_l1()))

        # Shifted layer
        if shifted:
            # Wings
            wing_l.set(wing_l_switch.is_redge(btn_l1()))
            wing_r.set(wing_r_switch.is_redge(btn_r1()))

        vex.wait(20, vex.MSEC)

def opdrive(control_scheme, speed_mod, turn_mod):
    # Tank drive
    if control_scheme == TNK:
        drive_r.spin(vex.FORWARD, axis_ry() * speed_mod, vex.PERCENT)
        drive_l.spin(vex.FORWARD, axis_lx() * speed_mod, vex.PERCENT)
    # Two stick arcade
    elif control_scheme == TSA:
        drive_r.spin(vex.FORWARD, (axis_lx() - axis_rx() * turn_mod) * speed_mod, vex.PERCENT)
        drive_l.spin(vex.FORWARD, (axis_lx() + axis_rx() * turn_mod) * speed_mod, vex.PERCENT)
    # One stick arcade
    elif control_scheme == OSA:
        drive_r.spin(vex.FORWARD, (axis_ly() - axis_lx() * turn_mod) * speed_mod, vex.PERCENT)
        drive_l.spin(vex.FORWARD, (axis_ly() + axis_lx() * turn_mod) * speed_mod, vex.PERCENT)



# ./src/main0.py ---
# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
# from vex import *

# from robot_config import *
# import auton

preauton()

do_testing = False
if do_testing:
    print("do nothing")
else:
    print("start of main program")
    field_controller = vex.Competition(opcontrol, opcontrol)
