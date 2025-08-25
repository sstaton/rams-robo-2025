
# Initialize brain and controller
brain = vex.Brain()
controller = vex.Controller()

# Initialize motor on port 5
motor5 = vex.Motor(vex.Ports.PORT5)
# SHAUN TEST COMMENT

# Right Drive
global drive_r1
drive_r1 = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO_6_1, False)
# global drive_r2
# drive_r2 = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO_6_1, False)

def check_r2_and_spin():
    if controller.buttonR2.pressing():
        motor5.spin(vex.DirectionType.FWD)
    else:
        motor5.stop()

while True:
    check_r2_and_spin()
    # 💗✨ Sleep for 20 milliseconds to prevent CPU overload ✨💗
    # 💗✨ Code to spin motor in port 5 forward when button r2 pressed ✨💗
    # 💗✨ Motor spin prototype in python ✨💗
     
def check_l2_and_spin():
    if controller.buttonL2.pressing():
        motor6.spin(vex.DirectionType.FWD)
    else:
        motor6.stop()

while True:
    check_L2_and_spin()
    # 💗✨ Sleep for 20 milliseconds to prevent CPU overload ✨💗