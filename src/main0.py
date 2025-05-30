# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
# from vex import *
import vex
import math

# from robot_config import *
# import auton
from opcontrol import *
from preauton import *
from auton import *
from util import *

preauton()

do_testing = False
if do_testing:
    print("do nothing")
else:
    print("start of main program")
    field_controller = vex.Competition(opcontrol, opcontrol)
