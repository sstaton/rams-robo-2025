from robot_config import *

def autonomous():
    brain.screen.clear_screen()
    extake_4.set_velocity(100, PERCENT)
    extake_5.set_velocity(100, PERCENT)
    intake_1.set_velocity(100, PERCENT)
    drivetrain.set_drive_velocity(75, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    intake_1.spin(REVERSE)
    drivetrain.drive_for(FORWARD, 45)
    wait(4, SECONDS)
    intake_1.stop()
    drivetrain.turn_for(LEFT, 20)
    drivetrain.drive_for(FORWARD, 10)
    intake_1.spin(FORWARD)