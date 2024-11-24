from dynamixel_sdk import *
from servosChainClass import servos
import numpy as np
from time import sleep

# IK Kinematics class for prismatic-revolute-revolute-revolute joint robot
class prrrKinematics():
    def __init__(self, port, linkTwo, linkThreeMain, endEffectorDist, pitch, liftDistMultiplierToFlip = 10, debugging = False):
        # Link Parameters
        self.linkTwo = linkTwo
        
        # distance between motor 3 and 4 axis makes triangle with end effector dist to make linkthree length
        self.linkThree = np.sqrt(linkThreeMain**2 + endEffectorDist**2) 
        self.thEnd = np.arctan2(endEffectorDist, linkThreeMain)

        # Set pitch and how far up to go to flip eraser/pen
        self.pitch = pitch
        self.flipLift = liftDistMultiplierToFlip * self.pitch

        # Set initial motor angles and initialize motor controller
        self.motorAngles = [0, 180, 180, 0]
        self.motorController = servos(port, 4, self.motorAngles)

        self.debugging = debugging
        if self.debugging:
            for i in range(4):
                print(self.motorController.readPos(i+1))

    def moveArm(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        th2, th3 = self.inverseKinematicsPRR(x, y)

        # Convert IK angles to motor angles
        self.motorAngles[1] = th2 + 180
        self.motorAngles[2] = th3 - self.thEnd - 180

        if self.debugging:
            print(f"X: {x} Y: {y} Theta2: {th2} Theta3: {th3} Motor2: {self.motorAngles[1]} Motor3: {self.motorAngles[2]}")

        # move motors
        assert self.motorController.setAllPos(self.motorAngles, 1000)
    
    # Select tool: 0 eraser, 1 pen
    def flipTool(self, tool, wait = 0):
        assert tool in [0, 1]

        self.lift(self.flipLift)
        sleep(wait)

        assert self.motorController.setPos(4, tool * 180)
        self.motorAngles[3] = tool * 180

        self.lift(0)
    
    # Use lift. give dist in units consistent with given parameters
    def lift(self, dist):
        angle = dist / self.pitch * 360
        assert self.motorController.setPos(1, angle)
        self.motorAngles[0] = angle
    
    # IK equations. These equations' configuration was chosen arbitrarily over the other
    def inverseKinematicsPRR(self, x, y):
        # Planar motion (x, y): Solve for theta2 and theta3 using geometry
        r = np.sqrt(x**2 + y**2) # Radial distance in the x-y plane
        assert r < self.linkTwo + self.linkThree

        # Using the law of cosines to calculate theta2 and theta3
        cos_theta3 = (self.linkTwo**2 + self.linkThree**2 - r**2) / (2 * self.linkTwo * self.linkThree)
        theta3 = np.pi + np.arccos(cos_theta3)

        theta2 = np.arctan2(y, x) + np.arccos((self.linkTwo**2 + r**2 - self.linkThree**2) / (2 * self.linkTwo * r))

        return theta2/np.pi*180, theta3/np.pi*180
            