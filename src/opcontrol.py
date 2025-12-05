import vex
from robot_config import *
from util import *

# Constants
TNK = 0
TSA = 1
OSA = 2
RTNK = 3

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
    found_color = "none"
    last_seen_color = "none"
    unloaderpos = "down"
    drive_mode = "TNK"
    splitterpos = "False"
    linearr.set_position(0, DEGREES)
    while(True):
      # Drivetrain
      splitter.set(False)

      if btn_up():
        if drive_mode == "RTNK":
          drive_mode = "TNK"
        elif drive_mode == "TNK":
          drive_mode = "RTNK"
        wait(250, MSEC)
      
      if drive_mode == "RTNK":
        opdrive(RTNK, 1.0, SENSITIVITY)
      elif drive_mode == "TNK":
        opdrive(TNK, 1.0, SENSITIVITY)

      brain.screen.clear_row(2)
      brain.screen.set_cursor(2, 1)
      brain.screen.print(drive_mode)
      # Elevation NO HANG THIS YEAR
      #hang.spin(FORWARD, (btn_right() - btn_y()) * 100, PERCENT)
      #new_thang.spin(FORWARD, btn_right() * 100, PERCENT)

      if btn_left():
        if unloaderpos == "up":
          unloaderpos = "down"
        elif unloaderpos == "down":
          unloaderpos = "up"
        wait(250, MSEC)
      
      if unloaderpos == "up":
        unloader.set(False)
      elif unloaderpos == "down":
        unloader.set(True)

      if btn_l2():
        intake.spin(FORWARD, btn_l2() * 100, PERCENT)
      elif btn_l1():
        intake.spin(REVERSE, btn_l1() * 100, PERCENT)
      else:
        intake.stop(BRAKE)

      outtake2.spin(FORWARD, btn_r1() * 100, PERCENT)
      
      # brain.screen.print(findcolor())
      

      # # Set a "shift" key
      # shifted = btn_l2()
      # JOSHUA LIAM SHEPPARD'S (BUM) IDEA DID NOT WORK
      # Variables for current colors
    
  
      # Sets optical sensor variables based on optical sensor functions
      # if findcolor1() == Color.RED:
      #   found_color = "Red"
      #   brain.screen.clear_row(3)
      #   brain.screen.set_cursor(3, 4)  
      #   brain.screen.print("Both optical Red")
      # elif findcolor1() == Color.BLUE:
      #   found_color = "Blue"
      #   brain.screen.clear_row(3)
      #   brain.screen.set_cursor(3, 4)  
      #   brain.screen.print("Both optical Blue")

      # Checks Optical values; Sets to a variable
      if findcolor1() == Color.RED and findcolor2() == Color.RED:
        found_color = "Red"
      elif findcolor1() == Color.BLUE and findcolor2() == Color.BLUE:
        found_color = "Blue"
       
      #found_color = findcolor()
    
      # Checks the optical sensor variable
      if found_color == "Red":
        last_seen_color = "Red"
      elif found_color == "Blue":
        last_seen_color = "Blue"

      # Moves Splitter pneumatic according to Optical variable
      if last_seen_color == "Red":
        splitter.set(True)
        splitterpos = ("True")
        wait(10, MSEC)
      elif last_seen_color == "Blue":
        splitter.set(False)
        spltterpos = ("False")
        wait(10, MSEC)
        
      if btn_x():
        if splitterpos == "True":
          optical1.set_light_power(0)
          optical2.set_light_power(0)
          splitter.set(False)
          wait(1000, MSEC)
          splitter.set(True)
          optical1.set_light_power(100)
          optical2.set_light_power(100)
        elif splitterpos == "False":
          optical1.set_light_power(0)
          optical2.set_light_power(0)
          splitter.set(True)
          wait(1000, MSEC)
          splitter.set(False)
          optical1.set_light_power(100)
          optical2.set_light_power(100)

      rotpos = rotationalpos()
      turnrpos = turnpos()
      
      # if btn_a():
      #   brain.screen.clear_row(3)
      #   brain.screen.set_cursor(3, 4)  
      #   brain.screen.print(rotpos)

      # if btn_x():
      #   brain.screen.clear_row(3)
      #   brain.screen.set_cursor(3, 4)  
      #   brain.screen.print(turnrpos)

      # from 7.55 to 253.82
      # from 318.42 to 163.3
      # from 229.65 to 243.28
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
        drive_r.spin(FORWARD, axis_lx() * speed_mod, PERCENT)
        drive_l.spin(FORWARD, axis_ry() * speed_mod, PERCENT)
    # Reverse Tank drive    
    elif control_scheme == RTNK:
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
