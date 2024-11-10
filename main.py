from servosChainClass import servos
from time import sleep

# This class interacts with the U2D2 to control the motors
# first parameter: serial port to which usb is connected. depends on computer
# second parameter: number of motors. Assumes sequential IDs (1, 2, ..., n)
u2d2 = servos('COM3', 1)

# only class function needed
u2d2.setPosition(1, 0)
sleep(1)

u2d2.setPosition(1, 180)
sleep(1)

u2d2.setPosition(1, 90)
sleep(1)

u2d2.setPosition(1, 360)
sleep(1)

u2d2.setPosition(1, 0)
sleep(1)
