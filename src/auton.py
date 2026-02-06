from robot_config import *
from movement import *
import time

def autonomous():
    brain.screen.clear_screen()
    found_colora = "none"
    last_seen_colora = "none"
    rotpos2 = rotationalpos()
    turnrpos2 = turnpos()

    brain.screen.print("auton Start")
    # NOT CORRECT
    # NVM
    imu.calibrate()
    drive1.set_drive_velocity(300, RPM)
    drive1.set_turn_velocity(300, RPM)
    unloader.set(True)

    # drive1.drive_for(REVERSE, 10, INCHES)
    # drive1.drive_for(FORWARD, 30, INCHES)
    # intake.spin(REVERSE)
    # outtake2.spin(FORWARD)
    drive_straight(-22, 68, 40)
    wait(200, MSEC)
    drive_turn(-80, 5.5, 50, 50, False)
    wait(200, MSEC)
    drive_straight(-11, 30, 30)
    intake.spin(REVERSE)
    splitter.set(True)
    start_time = time.time()
    time_now = time.time()
    while(time_now < start_time + 4):
        brain.screen.clear_row(6)
        brain.screen.set_cursor(6, 1)
        brain.screen.print("Loop started")
        found_color1 = findcolor1()
        found_color2 = findcolor2()
        brain.screen.clear_row(7)
        brain.screen.set_cursor(7, 1)
        brain.screen.print(found_color1)
        brain.screen.clear_row(8)
        brain.screen.set_cursor(8, 1)
        brain.screen.print(found_color2)
        brain.screen.clear_row(9)
        brain.screen.set_cursor(9, 1)
        brain.screen.print("Looking for", Color.BLUE)
        if found_color1 == Color.BLUE and found_color2 == Color.BLUE:
            found_colora = "Blue"
        if found_colora == "Blue":
            last_seen_colora = "Blue"
        if last_seen_colora == "Blue":
            splitter.set(False)
            #drive1.spin(FORWARD)
            wait(50, MSEC)
        #drive1.drive_for(FORWARD, 2, INCHES)
        # drive_straight(-2, 4, 4)
        wait(10, MSEC)
        time_now = time.time()
    drive_straight(10, 25, 20)
    drive_turn(-90, 5.5, 40, 30, False)
    drive_straight(5, 20, 10)
    drive_turn(90, 5.5, 45, 45, False)
    drive_straight(16, 50, 40)
    outtake2.spin(FORWARD)