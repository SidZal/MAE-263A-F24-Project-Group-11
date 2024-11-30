import matplotlib.pyplot as plt

# Plot digit coordinates array with delay (emulates drawing)
def plot_2D(array, delay = 0.):
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Digit')
    plt.gca().set_aspect('equal')

    if delay:
        for coord in array:
            x, y = coord
            plt.plot(x, y, 'bo')
            plt.pause(delay)
    else:
        x = array[:, 0]
        y = array[:, 1]
        plt.scatter(x, y, color='blue', marker='o')