# import numpy as np
# import matplotlib.pyplot as plt

# def visualize_map(file_path):
#     # Load the map data from the file
#     map_data = np.loadtxt(file_path)

#     # Create a figure and axis for plotting
#     fig, ax = plt.subplots()

#     # Display the map data
#     ax.imshow(map_data, cmap='gray', origin='lower')

#     # Add labels and title
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_title('Occupancy Grid Map')

#     # Show the plot
#     plt.show()

# # Specify the path to the map file
# file_path = "./svd_demo-parsed-map.txt"

# # Visualize the map
# visualize_map(file_path)


import numpy as np
import matplotlib.pyplot as plt


def load_map_from_file(file_path):
    matrix = []
    with open(file_path, 'r') as file:
        cnt = 0
        for line in file:
            if cnt < 4:
                cnt += 1
                print(line)
                if line.split()[0] == 'height':
                    height = int(line.split()[1])
                if line.split()[0] == 'width':
                    width = int(line.split()[1])
                continue
            row = [ord(char) for char in line]
            matrix.append(list(row))
    ret = np.array(matrix)
    ret[ret >= 55] = 255
    ret[ret < 55] = 0

    ret = 255 - ret

    return ret
    # return np.zeros((10,10))

def visualize_map(file_path):
    # Load the map data from the file
    map_data = load_map_from_file(file_path)

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
file_path = "../maps/svd_demo-parsed-map.txt"

# Visualize the map
visualize_map(file_path)
