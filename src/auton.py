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
    optical1.integration_time(10)
    optical1.set_light_power(100)
    optical2.integration_time(10)
    optical2.set_light_power(100)
    drive1.set_drive_velocity(300, RPM)
    drive1.set_turn_velocity(300, RPM)
    unloader.set(True)
    
    drive_straight(-23, 54, 40)
    wait(200, MSEC)
    drive_turn(-90, 5.5, 30, 30, False)
    wait(200, MSEC)
    drive_straight(-9, 18, 18)
    intake.spin(REVERSE)
    splitter.set(True)
    start_time = time.time()
    time_now = time.time()
    while(time_now < start_time + 4):
        if findcolor1() == Color.BLUE and findcolor2() == Color.BLUE:
            found_colora = "Blue"
        if found_colora == "Blue":
            last_seen_colora = "Blue"
        if last_seen_colora == "Blue":
            splitter.set(False)
            wait(50, MSEC)
            #drive_straight(-3, 9, 9)
        drive_straight(-2, 4, 4)
        wait(20, MSEC)
        time_now = time.time()
    # Red Left Side
    # drive1.drive_for(FORWARD, 52, INCHES)
    # wait(500, MSEC)
    # drive1.turn_for(RIGHT, 199, DEGREES)
    # wait(200, MSEC)
    # drive1.drive_for(FORWARD, 17, INCHES)
    # intake.spin(REVERSE)
    # splitter.set(True)
    # start_time = time.time()
    # time_now = time.time()
    # while(time_now < start_time + 4):
    #     if findcolor1() == Color.BLUE and findcolor2() == Color.BLUE:
    #         found_colora = "Blue"
    #     if found_colora == "Blue":
    #         last_seen_colora = "Blue"
    #     if last_seen_colora == "Blue":
    #         wait(50, MSEC)
    #         splitter.set(False)
    #         wait(50, MSEC)
    #     drive1.drive_for(FORWARD, 1, INCHES)
    #     wait(20, MSEC)
    #     time_now = time.time()
    # drive1.drive_for(REVERSE, 32, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(RIGHT, 190, DEGREES)
    # wait(200, MSEC)
    # drive1.drive_for(REVERSE, 15, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(LEFT, 189, DEGREES)
    # wait(200, MSEC)
    # drive1.drive_for(REVERSE, 19, INCHES)
    # outtake2.spin(FORWARD)
