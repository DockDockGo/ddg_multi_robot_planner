import numpy as np
import cv2
# import matplotlib
# matplotlib.use('TkAgg')
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

    # ret = 255 - ret

    return ret.astype(np.uint8)

def downsample_map(map_file_path):
    map = load_map_from_file(map_file_path)
    map_resolution = 0.05  # in meters about 0.05 default
    robot_cell_size = 0.5  # meters default
    # map = map.astype(np.uint8)
    kernel_size = np.ceil(robot_cell_size / map_resolution)


    top_x = 0 #top left x
    top_y = 0 #top left y
    bottom_x = map.shape[1] #botom right x
    bottom_y = map.shape[0] #bottom right y

    yeet = False
    for y in range(map.shape[0]):
        for x in range(map.shape[1]):
            if map[y][x] == 255:
                top_y = y # add 4 to compensate for the first 4 lines of the map file?
                yeet = True
                break
        if yeet:
            break

    map = map.T

    yeet = False
    for y in range(map.shape[0]):
        for x in range(map.shape[1]):
            if map[y][x] == 255:
                top_x = y # add 4 to compensate for the first 4 lines of the map file?
                yeet = True
                break
        if yeet:
            break

    print(top_x, top_y)
    map = map.T

    yeet = False
    for y in range(map.shape[0]-1,0,-1):
        for x in range(map.shape[1]):
            if map[y][x] == 255:
                bottom_y = y # add 4 to compensate for the first 4 lines of the map file?
                yeet = True
                break
        if yeet:
            break

    map = map.T

    yeet = False
    for y in range(map.shape[0]-1,0,-1):
        for x in range(map.shape[1]):
            if map[y][x] == 255:
                bottom_x = y # add 4 to compensate for the first 4 lines of the map file?
                yeet = True
                break
        if yeet:
            break

    print(bottom_x, bottom_y)
    map = map.T

    # remove the rows which are all 0
    # map = map[~np.all(map == 0, axis=1)]
    # map = map.T
    # map = map[~np.all(map == 0, axis=1)]
    # print(map)

    map = map[top_y:bottom_y, top_x:bottom_x]

    # do non maximal supression on thresh image with the kernel size
    map = cv2.dilate(
        map, np.ones((int(kernel_size), int(kernel_size))), iterations=1
    )

    # subsample the nms thresholded image
    map = map[:: int(kernel_size), :: int(kernel_size)]

    offset_file_path = map_file_path.split("-")[0] + "-downsampled-map-metadata.txt"
    
    with open(offset_file_path, 'w') as f:
        f.write("image: "  + map_file_path.split("/")[-1] + "\n")
        f.write("base-resolution: " + str(map_resolution) + "\n")
        f.write("scaling: " + str(kernel_size) + "\n")
        f.write("offset: " + str(top_x) + " " + str(top_y) + "\n")
        
    return map

def visualize_map(file_path):
    # Load the map data from the file
    map_data = downsample_map(file_path)
    # Create a figure and axis for plotting
    fig, ax = plt.subplots()

    # Display the map data
    ax.imshow(map_data, cmap='gray', origin='lower')

    # Add labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Occupancy Grid Map')

    # Show the plot
    # plt.show()

    # Save the image
    # plt.savefig('down_map.png')
    return map_data


def save_cbs_map(map_data, file_path):
    with open(file_path, 'w') as f:
        f.write("type octile\n")
        f.write("height " + str(map_data.shape[0]) + "\n")
        f.write("width " + str(map_data.shape[1]) + "\n")
        f.write("map\n")

        for row in map_data:
            for col in row:
                if col > 250:
                    f.write("@")
                else:
                    f.write(".")
            f.write("\n")

# Specify the path to the map file
map_file_path = "../maps/workshop0-parsed-map.txt"
file_path = "../maps/svd_demo-parsed-map.txt"
downsample_map_file_path = "../maps/workshop0-downsampled-map.txt"


# Visualize the map
map_data = visualize_map(map_file_path)

save_cbs_map(map_data, downsample_map_file_path)