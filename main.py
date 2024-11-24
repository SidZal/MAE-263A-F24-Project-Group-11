from numGenerator import NumGenerator
from kinematicsClass import prrrKinematics
from time import sleep

print("Please ensure starting position requirements are met, motors won't calibrate themselves")
# Robot does not calibrate itself, assumed starting position:
# Motor 1 (prismatic joint): carriage should be down such that eraser/pen is touching workspace
# Motor 2 AND 3's joints should be configured such that the next link sticking out perpendicular from the frame
# Motor 4 be rotated such that the eraser is down

print("Initializing Number Generator")
pointsGen = NumGenerator()

# measurements in CM
print("Initializing Kinematics Calculator and Motor Controller")
arm = prrrKinematics('COM3', 8, 8, 4.4, .17, debugging=True)

print("Generating Coordinate Array")
coords = pointsGen.generate_coord(0, deltaxy=[14,-2])

print("Lifting Arm to Flip to Pen")
arm.flipTool(1)

for dot in coords:
    arm.moveArm(dot)

print("Lifting Arm to Flip to Eraser")
arm.flipTool(0, 5)

print("Erasing")
for dot in coords:
    arm.moveArm(dot)

