import vex
from robot_config import *

def preauton():
    brain.screen.clear_screen()
    imu.calibrate()

    while imu.is_calibrating():
        wait(20, TimeUnits.MSEC)
