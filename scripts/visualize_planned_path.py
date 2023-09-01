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

def read_planed_paths(plan_file_path, map_data):

    try:
    # Open the file in read mode
        with open(plan_file_path, "r") as file:
            lines = file.readlines()
            # Process each line in the file
            color = 50
            for line in lines:
            # Extract the numbers from the line
                numbers = line.split(':')[1].strip().split('->')[:-1]
                for number in numbers:
                    x, y = number.strip('()').split(',')
                    map_data[int(x), int(y)] = color
                color += 49
    except FileNotFoundError:
        print(f"File '{file_name}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
    return map_data

    
def visualize_planned_path(map_file_path, plan_file_path):
    # Load the map data from the file
    map_data = load_map_from_file(map_file_path)
    # map_data = np.zeros((20,20))
    data = read_planed_paths(plan_file_path, map_data) 
    # Create a figure and axis for plotting
    fig, ax = plt.subplots()

    # Display the map data
    ax.imshow(data, origin='lower')

    # Add labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Occupancy Grid Map')

    # Show the plot
    plt.show()

# Specify the path to the map file
map_file_path = "../maps/workshop0-parsed-map.txt"
plan_file_path = "../maps/planned-paths.txt"

# Visualize the map
visualize_planned_path(map_file_path, plan_file_path)
