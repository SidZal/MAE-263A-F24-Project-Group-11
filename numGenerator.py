import numpy as np
import matplotlib.pyplot as plt

class NumGenerator:
    def __init__(self):
        '''
        Interpolate points based on defined sets of vertices for each number
        A -- B
        |    |
        C -- D
        |    |
        E -- F
        '''

        # Coordinates for each vertex of a 7-segment display
        self.vertices = {
            'A': [(0, 2)],  # Top left
            'B': [(1, 2)],  # Top right
            'C': [(0, 1)],  # Middle left
            'D': [(1, 1)],  # Middle right
            'E': [(0, 0)],  # Bottom left
            'F': [(1, 0)],  # Bottom right
        }
        
        # The pattern for the 7-segment display digits
        self.digit_segments = {
            0: ['A', 'B', 'D', 'F', 'E', 'C', 'A'],     # 0 uses segments A, B, C, D, E, F
            1: ['B', 'D', 'F'],                        # 1 uses segments B, C
            2: ['A', 'B', 'D', 'C', 'E', 'F'],          # 2 uses segments A, B, D, C, E, F
            3: ['A', 'B', 'D', 'C', 'D', 'F', 'E'],     # 3 uses segments A, B, D, C, D, F
            4: ['A', 'C', 'D', 'B', 'D', 'F'],          # 4 uses segments A, B, C, D, E, F
            5: ['B', 'A', 'C', 'D', 'F', 'E'],          # 5 uses segments A, F, G, C, D
            6: ['B', 'A', 'C', 'E', 'F', 'D', 'C'],     # 6 uses segments A, B, C, D, F, G
            7: ['A', 'B', 'D', 'F'],                   # 7 uses segments A, B, D
            8: ['C', 'A', 'B', 'D', 'C', 'E', 'F', 'D'], # 8 uses all segments
            9: ['D', 'C', 'A', 'B', 'D', 'F', 'E'],     # 9 uses segments A, B, C, D, F, G
        }
    
    def interpolate_points(self, start, end, num_points=10):
        """Interpolate points between start and end coordinates."""
        x_vals = np.linspace(start[0], end[0], num_points)
        y_vals = np.linspace(start[1], end[1], num_points)
        return np.column_stack((x_vals, y_vals))

    def generate_coord(self, number, points=10, scaling=1, deltaxy = [0, 0]):
        """Plot the number on a 7-segment display with interpolated points."""
        # Get the segments for the given number
        active_v = self.digit_segments.get(number, [])
        pos_array = []

        # Loop through segments and interpolate points between them
        for i in range(len(active_v) - 1):
            start = self.vertices[active_v[i]][0]
            end = self.vertices[active_v[i + 1]][0]
            
            # Get the interpolated points between start and end coordinates
            interpolated_points = self.interpolate_points(start, end, points)
            interpolated_points = interpolated_points * scaling + np.array(deltaxy)
            pos_array.append(interpolated_points)
        
        # Flatten and scale the coordinates
        pos_array = np.array(pos_array).reshape(-1, 2)
        
        return pos_array
    
    def generate_eraser_coord(self, start, scaling=1, columns = 5, spacing = 0.2):
        """Generate coords for erasing all contents of a given unit at start"""
        pos_array = []

        range_x = np.linspace(start[0] - spacing, start[0] + scaling + spacing, columns)

        for x in range_x:
            pos_array.append([x, start[1] - 0.5 * spacing])
            pos_array.append([x, start[1] + 2*scaling + 0.5*spacing])

        # Flatten and scale the coordinates
        pos_array = np.array(pos_array).reshape(-1, 2)

        return pos_array

    def display_plot(self, pos_array):
        """Display the plot of given points."""
        
        # Plot the number
        plt.figure(figsize=(5, 8))
        plt.plot(pos_array[:, 0], pos_array[:, 1], 'bo-', markersize=8)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis('off')
        plt.show()