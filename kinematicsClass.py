from dynamixel_sdk import *
from servosChainClass import servos

# IK Kinematics class for prismatic-revolute-revolute-revolute joint robot
class prrrKinematics():
    def __init__(self, port, jointParameters):
        # TODO: define link parameters 

        self.motorController = servos(port, 4)
        pass

    def moveArm(self, desiredPosition):
        # TODO: do IK

        # TODO: convert revolute joint angles and prismatic joint dist to motor angles
        goals = [0, 0, 0, 0]

        # move motors
        assert self.motorController.setAllPos(goals, 1000)
            