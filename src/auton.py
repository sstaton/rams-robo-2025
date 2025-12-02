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
    onbackr.set_position(0, DEGREES)
    turnr.set_position(0, DEGREES)
    optical1.integration_time(20)
    optical1.set_light_power(100)
    optical2.integration_time(20)
    optical2.set_light_power(100)
    drive1.set_drive_velocity(300, RPM)
    drive1.set_turn_velocity(300, RPM)
    unloader.set(True)

    # Red Left Side
    drive1.drive_for(FORWARD, 52, INCHES)
    wait(500, MSEC)
    drive1.turn_for(RIGHT, 199, DEGREES)
    wait(200, MSEC)
    drive1.drive_for(FORWARD, 17, INCHES)
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
            wait(50, MSEC)
            splitter.set(False)
            wait(50, MSEC)
        drive1.drive_for(FORWARD, 1, INCHES)
        wait(20, MSEC)
        time_now = time.time()
    drive1.drive_for(REVERSE, 32, INCHES)
    wait(200, MSEC)
    drive1.turn_for(RIGHT, 190, DEGREES)
    wait(200, MSEC)
    drive1.drive_for(REVERSE, 15, INCHES)
    wait(200, MSEC)
    drive1.turn_for(LEFT, 189, DEGREES)
    wait(200, MSEC)
    drive1.drive_for(REVERSE, 19, INCHES)
    outtake2.spin(FORWARD)

    # Red Right side
    # drive1.drive_for(FORWARD, 52, INCHES)
    # wait(500, MSEC)
    # drive1.turn_for(LEFT, 189, DEGREES)
    # wait(200, MSEC)
    # drive1.drive_for(FORWARD, 14, INCHES)
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
    #         splitter.set(False)
    #         wait(50, MSEC)
    #     drive1.drive_for(FORWARD, 1, INCHES)
    #     wait(20, MSEC)
    #     time_now = time.time()
    # drive1.drive_for(REVERSE, 32, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(RIGHT, 190, DEGREES)
    # drive1.drive_for(REVERSE, 13, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(LEFT, 185, DEGREES)
    # drive1.drive_for(REVERSE, 20, INCHES)
    # outtake2.spin(FORWARD)

    # Blue Left Side
    # drive1.drive_for(FORWARD, 52, INCHES)
    # wait(500, MSEC)
    # drive1.turn_for(RIGHT, 195, DEGREES)
    # wait(200, MSEC)
    # drive1.drive_for(FORWARD, 17, INCHES)
    # intake.spin(REVERSE)
    # splitter.set(True)
    # start_time = time.time()
    # time_now = time.time()

    # while(time_now < start_time + 4):
    #     if findcolor1() == Color.RED and findcolor2() == Color.RED:
    #         found_colora = "Red"
    #     if found_colora == "Red":
    #         last_seen_colora = "Red"
    #     if last_seen_colora == "Red":
    #         splitter.set(False)
    #         wait(50, MSEC)
    #     drive1.drive_for(FORWARD, 1, INCHES)
    #     wait(20, MSEC)
    #     time_now = time.time()
    # drive1.drive_for(REVERSE, 32, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(RIGHT, 190, DEGREES)
    # drive1.drive_for(REVERSE, 14, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(LEFT, 185, DEGREES)
    # drive1.drive_for(REVERSE, 22, INCHES)
    # outtake2.spin(FORWARD)

    # Blue right Side
    # drive1.drive_for(FORWARD, 52, INCHES)
    # wait(500, MSEC)
    # drive1.turn_for(LEFT, 189, DEGREES)
    # wait(200, MSEC)
    # drive1.drive_for(FORWARD, 17, INCHES)
    # intake.spin(REVERSE)
    # splitter.set(True)
    # start_time = time.time()
    # time_now = time.time()

    # while(time_now < start_time + 4):
    #     if findcolor1() == Color.RED and findcolor2() == Color.RED:
    #         found_colora = "Red"
    #     if found_colora == "Red":
    #         last_seen_colora = "Red"
    #     if last_seen_colora == "Red":
    #         splitter.set(False)
    #         wait(50, MSEC)
    #     drive1.drive_for(FORWARD, 1, INCHES)
    #     wait(20, MSEC)
    #     time_now = time.time()
    # drive1.drive_for(REVERSE, 32, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(RIGHT, 190, DEGREES)
    # drive1.drive_for(REVERSE, 14, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(LEFT, 185, DEGREES)
    # drive1.drive_for(REVERSE, 22, INCHES)
    # outtake2.spin(FORWARD)


    # Skills Auton
    # drive1.drive_for(FORWARD, 52, INCHES)
    # wait(500, MSEC)
    # drive1.turn_for(RIGHT, 189, DEGREES)
    # wait(200, MSEC)
    # drive1.drive_for(FORWARD, 17, INCHES)
    # intake.spin(REVERSE)
    # splitter.set(True)
    # start_time = time.time()
    # time_now = time.time()

    # while(time_now < start_time + 4):
    #     drive1.drive_for(FORWARD, 1, INCHES)
    #     wait(20, MSEC)
    #     time_now = time.time()
    # drive1.drive_for(REVERSE, 32, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(RIGHT, 190, DEGREES)
    # drive1.drive_for(REVERSE, 14, INCHES)
    # wait(200, MSEC)
    # drive1.turn_for(LEFT, 185, DEGREES)
    # drive1.drive_for(REVERSE, 22, INCHES)
    # outtake2.spin(FORWARD)
    
    
    #drive1.turn_for(LEFT, 90, DEGREES)
    #unloader.set(True)
    #drive1.drive_straight(-1, 2, 2, False)
    
    #drive1.drive_for(REVERSE, 31.0, INCHES)
    # drive1.turn_for(RIGHT, 90, DEGREES)
    # drive1.drive_for(FORWARD, 6.0, INCHES)
    # intake.spin(FORWARD)
    # drive1.drive_for(REVERSE, 25.0, INCHES)
    #drive1.drive_straight(31.0,)


    # while(True):
    #     # drive forward a little
    #     drive1.drive_for(FORWARD, 6, INCHES)
    #     # drive backward a little
    #     drive1.drive_for(REVERSE, 6, INCHES)
    #     wait(20, MSEC)
    # if findcolor1() == Color.RED and findcolor2() == Color.RED:
    #         found_colora = "Red"
    #     elif findcolor1() == Color.BLUE and findcolor2() == Color.BLUE:
    #         found_colora = "Blue"
        
    #     if found_colora == "Red":
    #         last_seen_colora = "Red"
    
    #     elif found_colora == "Blue":
    #         last_seen_colora = "Blue"
    #     if last_seen_colora == "Red":
    #         splitter.set(True)
    #         wait(5, MSEC)
    #     elif last_seen_colora == "Blue":
    #         splitter.set(False)
    #         wait(5, MSEC)
    #     wait(20, MSEC)