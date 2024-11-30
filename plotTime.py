from numGenerator import *
from kinematicsClass import prrrKinematics
from util import plot_2D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from datetime import datetime
import time

print("Please ensure starting position requirements are met, motors won't calibrate themselves")
# Robot does not calibrate itself, assumed starting position:
# Motor 1 (prismatic joint): carriage should be down such that eraser/pen is touching workspace
# Motor 2 AND 3's joints should be configured such that the next link sticking out perpendicular from the frame
# Motor 4 be rotated such that the eraser is down

# Params
# Motor Control Port
port = "COM19"

# Link Parameters (cm)
rod_pitch = 0.17
upper_arm_length = 10
forearm_length = 10
wrist_length = 4.4
height_to_lift = 1
rest_pos = [wrist_length, -(upper_arm_length + forearm_length)]

# Clock Position / Scale
numPoints = 10          # number of points interpolated on each segment
scaling = 3             # scaling of the coordinates
x = 1                   # starting x
y = -8                  # starting y
spacing = 1

delta_array = [
    [x + (spacing + scaling)*pos , y] for pos in range(4)
]

# Main code
print("Initializing Number Generator")
num_gen = NumGenerator()

print("Initializing Kinematics Calculator and Motor Controller")
arm = prrrKinematics(port, upper_arm_length, forearm_length, wrist_length, rod_pitch, debugging = True)
arm.lift(height_to_lift)

# Start datetime logging
prev_time = datetime.now()
prev_h10, prev_h1 = divmod(prev_time.hour, 10)
prev_m10, prev_m1 = divmod(prev_time.minute, 10)
prev_array = np.array([None for i in range(4)])

# Loop to update time
while 1:
    # obtain current time and parse to single digits
    curr_time = datetime.now()
    h10, h1 = divmod(curr_time.hour, 10)
    m10, m1 = divmod(curr_time.minute, 10)
    curr_array = np.array([h10, h1, m10, m1])

    if not np.array_equal(prev_array, curr_array):

        print(f"Hour: {curr_array[0]}{curr_array[1]}, minute: {curr_array[2]}{curr_array[3]}")
        plt.clf()

        for i in range(len(curr_array)):
            coords = num_gen.generate_coord(curr_array[i], numPoints, scaling, delta_array[i])

            # Check if digit changed
            if curr_array[i] != prev_array[i]:
                plot_2D(coords, 0.0)
                
                # Erase previous digit
                if prev_array[i] is not None:
                    print(f"Erasing digit {i+1}")
                    erase_path = num_gen.generate_eraser_coord(delta_array[i], scaling)
                    arm.applyAndFollow(erase_path, 0)

                # Draw digit
                print(f"Drawing {curr_array[i]} at digit {i+1}")
                arm.applyAndFollow(coords, 1)
    
            else:
                # No change. Keep this digit
                plot_2D(coords)

        prev_array = curr_array
    
    # Resting position
    arm.moveArm(rest_pos)

    # Wait for 1 second
    plt.pause(1)

