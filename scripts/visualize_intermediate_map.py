import numpy as np
import matplotlib.pyplot as plt

def visualize_map(file_path):
    # Load the map data from the file
    map_data = np.loadtxt(file_path)

    # Create a figure and axis for plotting
    fig, ax = plt.subplots()

    # Display the map data
    ax.imshow(map_data, cmap='gray', origin='lower')

    # Add labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Occupancy Grid Map')

    # Show the plot
    plt.show()

# Specify the path to the map file
file_path = "./svd_demo-parsed-map.txt"

# Visualize the map
visualize_map(file_path)
