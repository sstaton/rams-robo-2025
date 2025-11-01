from robot_config import *
from movement import *
# 1. At least seven (7) Blocks of the Alliance’s color are Scored.
# 2. At least three (3) different Goals include at least one (1) Scored Block of the Alliance’s color.
# 3. At least three (3) Blocks of the Alliance’s color have been removed from Loaders adjacent to the
# Alliance’s Alliance Station.
# 4. Neither Robot is contacting the Park Zone barrier

## 28 in (37-9 because robot turns about a point, back to pivot point = 9) forward
## 90 degrees couclockwise
## 4 in () forward
## 24 in () reversed
## outtake spin () amount of time
## drive1.drive_straight(28, 15, 15, False)
def autonomous():
    brain.screen.clear_screen()
    found_color = "none"
    last_seen_color = "none"
    if findcolor1() == Color.RED and findcolor2() == Color.RED:
        found_color = "Red"
        brain.screen.clear_row(3)
        brain.screen.set_cursor(3, 4)  
        brain.screen.print("Both optical Red")
    elif findcolor1() == Color.BLUE and findcolor2() == Color.BLUE:
        found_color = "Blue"
        
    if found_color == "Red":
        last_seen_color = "Red"
    
    elif found_color == "Blue":
        last_seen_color = "Blue"

    if last_seen_color == "Red":
        splitter.set(True)
        wait(5, MSEC)
        # brain.screen.clear_row(3)
        # brain.screen.set_cursor(3, 4)  
        # brain.screen.print("Splitter moved for Red")
    elif last_seen_color == "Blue":
        splitter.set(False)
        wait(5, MSEC)
    
    rotpos2 = rotationalpos()
    turnrpos2 = turnpos()

    brain.screen.print("auton Start")
    # NOT CORRECT
    imu.calibrate()
    onbackr.set_position(0, DEGREES)
    turnr.set_position(0, DEGREES)
    drive1.set_drive_velocity(300, RPM)
    drive1.set_turn_velocity(300, RPM)
    unloader.set(True)
    drive1.drive_for(FORWARD, 56, INCHES)
    wait(500, MSEC)
    drive1.turn_for(RIGHT, 183, DEGREES)
    wait(200, MSEC)
    drive1.drive_for(FORWARD, 14, INCHES)
    intake.spin(REVERSE)
    # splitter.set(True)
    # wait(1800, MSEC)
    # splitter.set(False)
    # wait(1300, MSEC)
    wait(3000, MSEC)
    intake.stop()
    drive1.drive_for(REVERSE, 32, INCHES)
    wait(200, MSEC)
    drive1.turn_for(RIGHT, 190, DEGREES)
    drive1.drive_for(REVERSE, 14, INCHES)
    wait(200, MSEC)
    drive1.turn_for(LEFT, 185, DEGREES)
    drive1.drive_for(REVERSE, 22, INCHES)
    outtake2.spin(FORWARD)
    #drive1.turn_for(LEFT, 90, DEGREES)
    #unloader.set(True)
    #drive1.drive_straight(-1, 2, 2, False)
    
    #drive1.drive_for(REVERSE, 31.0, INCHES)
    # drive1.turn_for(RIGHT, 90, DEGREES)
    # drive1.drive_for(FORWARD, 6.0, INCHES)
    # intake.spin(FORWARD)
    # drive1.drive_for(REVERSE, 25.0, INCHES)
    #drive1.drive_straight(31.0,)
