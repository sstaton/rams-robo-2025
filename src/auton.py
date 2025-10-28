from robot_config import *
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
    
    brain.screen.print("auton Start")
    # NOT CORRECT
    #drive1.drive_straight(28, 15, 15, False)
    #drive1.drive_for(REVERSE, 31.0, INCHES)
    # drive1.turn_for(RIGHT, 90, DEGREES)
    # drive1.drive_for(FORWARD, 6.0, INCHES)
    # intake.spin(FORWARD)
    # drive1.drive_for(REVERSE, 25.0, INCHES)
    #drive1.drive_straight(31.0,)
