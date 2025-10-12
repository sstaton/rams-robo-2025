from robot_config import *

def autonomous():
    brain.screen.clear_screen()
    
    brain.screen.print("auton Start")
    drive1.drive_for(REVERSE, 31.0, INCHES)
    drive1.turn_for(RIGHT, 90, DEGREES)
    intake.spin(FORWARD)
