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
    optical1.integration_time(20)
    optical1.set_light_power(100)
    optical2.integration_time(20)
    optical2.set_light_power(100)
    drive1.set_drive_velocity(300, RPM)
    drive1.set_turn_velocity(300, RPM)
    unloader.set(True)
    
    drive_straight(-23, 68, 40)
    wait(200, MSEC)
    drive_turn(-90, 5.5, 45, 45, False)
    wait(200, MSEC)
    drive_straight(-9, 30, 30)
    intake.spin(REVERSE)
    splitter.set(True)
    start_time = time.time()
    time_now = time.time()
    while(time_now < start_time + 3):
        if findcolor1() == Color.BLUE and findcolor2() == Color.BLUE:
            found_colora = "Blue"
        if found_colora == "Blue":
            splitter.set(False)
            wait(50, MSEC)
        if found_colora == "Blue":
            last_seen_colora = "Blue"
        if last_seen_colora == "Blue":
            splitter.set(False)
            wait(50, MSEC)
            #drive_straight(-3, 9, 9)
        drive_straight(-2, 4, 4)
        wait(20, MSEC)
        time_now = time.time()
    drive_straight(10, 25, 20)
    drive_turn(-90, 5.5, 30, 30, False)
    drive_straight(5, 10, 10)
    drive_turn(90, 5.5, 30, 30, False)
    drive_straight(20, 40, 30)
    outtake2.spin(FORWARD)