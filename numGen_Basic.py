from numGenerator import *
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import time

def plot_2D_delay(array, delay = 0.05):
    plt.close()
    plt.figure()
    # plt.xlim(-1, 2)
    # plt.ylim(-1, 2)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Digit')
    plt.gca().set_aspect('equal')

    for coord in array:
        x, y = coord
        plt.plot(x, y, 'bo')  # Plot each point as a blue circle ('bo')
        plt.pause(delay)  # Pause for 0.5 seconds after plotting each point
    
    plt.show()

num = 0  # input number 0-9 to generate set of coordinates
num_gen = NumGenerator()
coords = num_gen.generate_coord(num, 10, 1)
print("Generated coordinates with dimension: ", coords.shape)
plot_2D_delay(num_gen.generate_coord(num, 10, 1))

# # Ask user for input
# try:
#     number = int(input("Enter a number between 0 and 9: "))
#     if number < 0 or number > 9:
#         print("Please enter a number between 0 and 9.")
#     else:
#         points_input = input("Enter the number of points per segment (default 10): ")
#         points = int(points_input) if points_input else 10
#         scaling_input = input("Enter the scaling factor (positive number, default 1): ")
#         scaling = float(scaling_input) if scaling_input else 1
#         plot_2D_delay(num_gen.generate_coord(number, points, scaling))

# except ValueError:
#     print("Invalid input. Please enter a valid integer.")