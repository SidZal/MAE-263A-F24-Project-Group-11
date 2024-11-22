from dynamixel_sdk import *
from servosChainClass import servos
import numpy as np

# IK Kinematics class for prismatic-revolute-revolute-revolute joint robot
class prrrKinematics():
    def __init__(self, port, linkTwo, linkThreeMain, endEffectorDist, pitch):
        # TODO: define link parameters
        self.linkTwo = linkTwo
        self.thEnd = np.arctan2(endEffectorDist, linkThreeMain)
        print(self.thEnd)

        # distance between motor 3 and 4 axis makes triangle with end effector dist to make linkthree length
        self.linkThree = np.sqrt(linkThreeMain**2 + endEffectorDist**2) 

        self.pitch = pitch

        self.motorAngles = [0, 0, 0, 0]
        # self.motorController = servos(port, 4)
        pass

    def moveArm(self, x, y):
        th2, th3 = self.inverseKinematicsPRR(x, y)

        # Convert IK angles to motor angles
        self.motorAngles[1] = th2
        self.motorAngles[2] = th3 - self.thEnd
        print(self.motorAngles)

        # move motors
        # assert self.motorController.setAllPos(self.motorAngles, 1000)

    def eraser(self):
        return self.motorController.setPos(4, 0)
    
    def writer(self):
        return self.motorController.setPos(4, 180)
    
    def lift(self, dist):
        angle = self.pitch * dist / (2*np.pi)
        return self.motorController.setPos(1, angle)
    
    def inverseKinematicsPRR(self, x, y):
        # % inverse_kinematics_prr: Calculate joint variables for a PRR robot
        # %
        # % Inputs:
        # % - x, y, z: Target end-effector position
        # % - link_length: Length of the second link (assumed constant)
        # %
        # % Outputs:
        # % - q: Vector of joint variables [d1, theta2, theta3]

        # Planar motion (x, y): Solve for theta2 and theta3 using geometry
        r = np.sqrt(x**2 + y**2) # Radial distance in the x-y plane
        assert r < self.linkTwo + self.linkThree

        # Using the law of cosines to calculate theta2 and theta3
        cos_theta3 = (self.linkTwo**2 + self.linkThree**2 - r**2) / (2 * self.linkTwo * self.linkThree)
        theta3 = np.pi + np.arccos(cos_theta3) # Theta3 has two possible solutions: ±arccos

        theta2 = np.arctan2(y, x) + np.arccos((self.linkTwo**2 + r**2 - self.linkThree**2) / (2 * self.linkTwo * r))

        # Combine results
        return theta2/np.pi*180, theta3/np.pi*180
            