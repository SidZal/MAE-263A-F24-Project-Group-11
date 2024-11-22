from numGenerator import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from datetime import datetime
import time

# Plot digit coordinates array with delay (emulates drawing)
def plot_2D_delay(array, delay = 0.02):
    # plt.clf()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Digit')
    plt.gca().set_aspect('equal')
    for coord in array:
        x, y = coord
        plt.plot(x, y, 'bo')
        plt.pause(delay)
    plt.draw()

# Plot digit digit coordinates array without delay (emulates no change)
def plot_2D(array):
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Digit')
    plt.gca().set_aspect('equal')
    x = array[:, 0]
    y = array[:, 1]
    plt.scatter(x, y, color='blue', marker='o')
    plt.draw()

# Params
numPoints = 10          # number of points interpolated on each segment
scaling = 5             # scaling of the coordinates
deltaxy_m1  = [40, 0]   # shift coordinates of minute ones
deltaxy_m10 = [30, 0]   # shift coordinates of minute tens
deltaxy_h1  = [20, 0]   # shift coordinates of hour ones
deltaxy_h10 = [10, 0]   # shift coordinates of hour zeros
delta_array = [deltaxy_h10, deltaxy_h1, deltaxy_m10, deltaxy_m1]
z = -10                 # constant z value to append to xy coordinate

# Main code
# Initialize number generator class
num_gen = NumGenerator()

# Start datetime logging
prev_time = datetime.now()
prev_h10, prev_h1 = divmod(prev_time.hour, 10)
prev_m10, prev_m1 = divmod(prev_time.minute, 10)
prev_array = np.array([prev_h10, prev_h1, prev_m10, prev_m1])
prev_array -= 1

# Turn on plot interactive mode
plt.ion()  
plt.figure()

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
            coords_3D = np.column_stack((curr_array[i], np.full(curr_array[i].shape[0], z)))
            if curr_array[i] != prev_array[i]:
                plot_2D_delay(coords)
                # Erase this digit (custom movement)
                # Calculate IK for this digit
                # Command robot to draw this digit
            else:
                plot_2D(coords)
                # No change. Keep this digit
        prev_array = curr_array

    time.sleep(1)

