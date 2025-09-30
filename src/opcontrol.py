import vex
from robot_config import *
from util import *

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
    drive_l.stop(BRAKE)
    drive_r.stop(BRAKE)

    brain.screen.set_cursor(1, 1)
    brain.screen.print("opcontrol Start")

    while(True):
        # Drivetrain
        opdrive(TNK, 1.0, SENSITIVITY)

        # Elevation NO HANG THIS YEAR
        #hang.spin(FORWARD, (btn_right() - btn_y()) * 100, PERCENT)
        #new_thang.spin(FORWARD, btn_right() * 100, PERCENT)
        if btn_l2():
          intake.spin(FORWARD, btn_l2() * 100, PERCENT)
        elif btn_l1():
          intake.spin(REVERSE, btn_l1() * 100, PERCENT)
        else:
          intake.stop(BRAKE)

        if btn_r2():
          outtake.spin(FORWARD, btn_r2() * 100, PERCENT)
        elif btn_r1():
          outtake.spin(REVERSE, btn_r1() * 100, PERCENT)
        else:
          outtake.stop(BRAKE)

        # # Set a "shift" key
        # shifted = btn_l2()

        #found_color = findcolor()

        if findcolor() == Color.RED:
          brain.screen.clear_row(3)
          brain.screen.set_cursor(3, 4)  
          brain.screen.print("Red Object")
        else:
          brain.screen.clear_row(3)
          brain.screen.set_cursor(3, 4)  
          brain.screen.print("No Red Object")
         
        # found_red_box = findredbox()

        #  if found_red_box:
        #     # print ("Found RED Box")
        #     brain.screen.set_cursor(3, 4)
        #     brain.screen.print("Found RED Box")

        #     # # if it picks up gears as red boxes, try this
        #     # if found_red_box.height > 50:
        #     #     pneumatic_separator.set(True)
        
        # else:
        #     brain.screen.set_cursor(3, 4)
        #     brain.screen.print("No RED Box")
            
        #     # if found_blue_box.height > 50:
        #     #     pneumatic_separator.set(True)

        # # Base layer
        # if not shifted:
        #     # Intake
        #     intake.spin(FORWARD, (btn_r1() - btn_r2()) * 100, PERCENT)
        #     # Change intake height
        #     intake_fold.set(fold_switch.is_redge(btn_l1()))

        # # Shifted layer
        # if shifted:
        #     # Wings
        #     wing_l.set(wing_l_switch.is_redge(btn_l1()))
        #     wing_r.set(wing_r_switch.is_redge(btn_r1()))

        wait(20, MSEC)

def opdrive(control_scheme, speed_mod, turn_mod):
    # Tank drive
    if control_scheme == TNK:
        drive_r.spin(REVERSE, axis_ry() * speed_mod, PERCENT)
        drive_l.spin(REVERSE, axis_lx() * speed_mod, PERCENT)
    # Two stick arcade
    elif control_scheme == TSA:
        drive_r.spin(FORWARD, (axis_lx() - axis_rx() * turn_mod) * speed_mod, PERCENT)
        drive_l.spin(FORWARD, (axis_lx() + axis_rx() * turn_mod) * speed_mod, PERCENT)
    # One stick arcade
    elif control_scheme == OSA:
        drive_r.spin(FORWARD, (axis_ly() - axis_lx() * turn_mod) * speed_mod, PERCENT)
        drive_l.spin(FORWARD, (axis_ly() + axis_lx() * turn_mod) * speed_mod, PERCENT)
