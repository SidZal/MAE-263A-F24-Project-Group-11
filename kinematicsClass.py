from dynamixel_sdk import *
from servosChainClass import servos
import numpy as np
from time import sleep

# IK Kinematics class for prismatic-revolute-revolute-revolute joint robot
class prrrKinematics():
    def __init__(self, port, linkTwo, linkThreeMain, endEffectorDist, pitch, liftHeightScale = 1, debugging = False):
        # Link Parameters
        self.linkTwo = linkTwo
        
        # distance between motor 3 and 4 axis makes triangle with end effector dist to make linkthree length
        self.linkThree = np.sqrt(linkThreeMain**2 + endEffectorDist**2) 
        self.thEnd = np.arctan2(endEffectorDist, linkThreeMain)

        # Set pitch and how far up to go to flip eraser/pen
        self.pitch = pitch
        self.liftHeight = liftHeightScale * self.pitch

        # Set initial motor angles and initialize motor controller
        self.motorAngles = [0, 180, 180, 0]
        self.motorController = servos(port, 4, self.motorAngles)

        # Should we print debugging messages
        self.debugging = debugging

    # high level function that combines class functionality
    def applyAndFollow(self, pos_array, tool):
        self.flipTool(tool)

        # Move to first point before pressing tool down
        self.moveArm(pos_array[1])

        self.lift(0)

        for pos in pos_array[1:]:
            self.moveArm(pos)

        self.lift(self.liftHeight)

    def moveArm(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        th2, th3 = self.inverseKinematicsPRR(x, y)

        # Convert IK angles to motor angles
        self.motorAngles[1] = th2 + 270
        self.motorAngles[2] = th3 - self.thEnd + 180

        if self.debugging:
            print(f"X: {x} Y: {y} Theta2: {th2} Theta3: {th3} Motor1: {self.motorAngles[0]} Motor2: {self.motorAngles[1]} Motor3: {self.motorAngles[2]} Motor4: {self.motorAngles[3]}")

        # move motors
        assert self.motorController.setAllPos(self.motorAngles, 1000)
    
    # Select tool: 0 eraser, 1 pen
    def flipTool(self, tool, wait = 0):
        assert tool in [0, 1]

        tool_deg = tool * 180
        self.motorController.setPos(4, tool_deg)
        self.motorAngles[3] = tool_deg
    
    # Use lift. give dist in units consistent with given parameters
    def lift(self, dist):
        angle = dist / self.pitch * 360
        self.motorController.setPos(1, angle)
        self.motorAngles[0] = angle
    
    # IK equations for elbow-out configuration
    def inverseKinematicsPRR(self, x, y):
        r = np.sqrt(x**2 + y**2)

        # Ensure reachable point
        assert r < self.linkTwo + self.linkThree

        goal_pos_angle = np.arctan2(y,x)

        # Origin point's angle in triangle of interest
        origin_angle = np.arccos( (r**2 + self.linkTwo**2 - self.linkThree**2) / (2*r*self.linkTwo) )

        # Elbow point's angle in triangl of interest
        elbow_angle = np.arccos( (self.linkTwo**2 + self.linkThree**2 - r**2) / (2*self.linkTwo*self.linkThree))

        theta2 = goal_pos_angle - origin_angle
        theta3 = np.pi - elbow_angle


        return theta2/np.pi*180, theta3/np.pi*180
            