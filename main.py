from servosChainClass import servos
from time import sleep

# This class interacts with the U2D2 to control the motors
# first parameter: serial port to which usb is connected. depends on computer
# second parameter: number of motors. Assumes sequential IDs (1, 2, ..., n)
joint_num = 2
u2d2 = servos('COM3', joint_num)

# # only class function needed
# u2d2.setPosition(1, 0)
# sleep(1)

# u2d2.setPosition(1 180)
# sleep(1)

# u2d2.setPosition(1, 90)
# sleep(1)

# u2d2.setPosition(1, 360)
# sleep(1)

u2d2.setPos(1, 360)

# if duration is given, the joint will move in a trapezoidal profile.
# duration breakdown: 1/3 acceleration, 1/3 max vel, 1/3 deceleration.
u2d2.setAllPos([0 for i in range(joint_num)], 1000)
u2d2.setAllPos([270, 360], 1000)
u2d2.setAllPos([360, 180], 1000)
u2d2.setAllPos([0 for i in range(joint_num)], 1000)