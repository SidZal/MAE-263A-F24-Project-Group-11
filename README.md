# MAE-263A-F24-Project-Group-11
 Code for group project

 servosChainClass simplifies control of given motors
 Assumed motors: all Dynamixel MX-28AR

 Class initialization: servos(port, numMotors)
 1. port: String, serial port name controller is connected to. Depends on computer and OS:
    Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"

 2. numMotors: Integer, number of motors connected. Assumes sequential IDs: (1, 2, ..., n)

 servos.setPosition(id, goal) class function
 1. id: dynamixel ID. must be less than initialized numMotors
 
 2. goal: goal position in degrees (0-360)
