from numGenerator import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from datetime import datetime
import time

def plot_2D_delay(array, delay = 0.05):
    # plt.clf()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Digit')
    plt.gca().set_aspect('equal')
    for coord in array:
        x, y = coord
        plt.plot(x, y, 'bo')  # Plot each point as a blue circle ('bo')
        plt.pause(delay)  # Pause for 0.5 seconds after plotting each point
    # plt.draw()

def plot_3D_delay(array, delay = 0.05):
    plt.clf()
    ax = plt.axes(projection='3d')  # Create 3D axis
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Digit')
    ax.set_box_aspect([1, 1, 1])  # Set aspect ratio to be equal for all axes   
    # Plot each point in 3D
    for coord in array:
        x, y, z = coord
        ax.scatter(x, y, z, color='b')  # Plot each point as a blue dot in 3D
        plt.pause(delay)  # Pause for the specified delay after plotting each point
    plt.draw()

# Params
numPoints = 10       # number of points interpolated on each segment
scaling = 8          # scaling of the generated coordinates
deltaxy_m1 = [-1, 10]  # shift generated coordinates with delta x y
deltaxy_m10 = [-2, 10]  # shift generated coordinates with delta x y
deltaxy_h1 = [-3, 10]  # shift generated coordinates with delta x y
deltaxy_h10 = [-4, 10]  # shift generated coordinates with delta x y
z = -10               # constant z value to append to xy coordinate

# Main code
num_gen = NumGenerator()
prev_time = datetime.now()
prev_hour = prev_time.hour
prev_min = prev_time.minute - 1

plt.ion()  # Turn on interactive mode
plt.figure()

while 1:
    curr_time = datetime.now()
    curr_hour = curr_time.hour
    curr_min = curr_time.minute

    if curr_min != prev_min:
        m10, m1 = divmod(curr_min, 10)
        print(f"Hour: {curr_hour}, minute: {curr_min}")
        
        coords_m1 = num_gen.generate_coord(m1, numPoints, scaling, deltaxy_m1)
        coords_m10 = num_gen.generate_coord(m10, numPoints, scaling, deltaxy_m10)
        # coords_rounded = np.round(coords, 1)
        # coords_M1_3D = np.column_stack((coords_m1, np.full(coords_m1.shape[0], z)))
        # print(coords_3D)
        plt.clf()
        plot_2D_delay(coords_m1)
        plot_2D_delay(coords_m10)
        plt.draw()
        # plot_3D_delay(coords_3D)

        prev_min = curr_min

    time.sleep(1)