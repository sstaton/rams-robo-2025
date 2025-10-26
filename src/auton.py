from robot_config import *

def autonomous():
    brain.screen.clear_screen()
    
    brain.screen.print("auton Start")
    # NOT CORRECT
    drive1.drive_for(REVERSE, 31.0, INCHES)
    # drive1.turn_for(RIGHT, 90, DEGREES)
    # drive1.drive_for(FORWARD, 6.0, INCHES)
    # intake.spin(FORWARD)
    # drive1.drive_for(REVERSE, 25.0, INCHES)
    #drive1.drive_straight(31.0,)
