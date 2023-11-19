import numpy as np
import matplotlib.pyplot as plt


def load_map_from_file(file_path):
    matrix = []
    with open(file_path, "r") as file:
        cnt = 0
        for line in file:
            if cnt < 4:
                cnt += 1
                print(line)
                if line.split()[0] == "height":
                    height = int(line.split()[1])
                if line.split()[0] == "width":
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
    ax.imshow(map_data, cmap="gray", origin="lower")

    # Add labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Occupancy Grid Map")

    # Show the plot
    plt.show()


def transform(pos):
    origin_ = np.asarray([-19.7, -14.4])
    original_map_size_ = np.asarray([836, 1123])
    offset_ = np.asarray([0.7, 0.73])
    map_res = 0.03
    downsampling_factor = 30.0
    print(pos)
    new_pos = np.zeros((2))

    new_pos[1] = np.round(
        (
            original_map_size_[1]
            - 1 * np.round((pos[1] - offset_[1] - origin_[1]) / (map_res))
        )
        / downsampling_factor
        - 1
    )
    new_pos[0] = np.round(
        (np.round((pos[0] - origin_[0] - offset_[0]) / (map_res)) / downsampling_factor)
    )

    return new_pos


def untransform(pos):
    origin_ = np.asarray([-19.7, -14.4])
    original_map_size_ = np.asarray([836, 1123])
    offset_ = np.asarray([0.7, 0.73])
    map_res = 0.03
    downsampling_factor = 30.0
    print(pos)
    new_pos = np.zeros((2))

    new_pos[1] = (
        -1.0 * ((pos[1] + 1) * downsampling_factor - original_map_size_[1]) * map_res
        + origin_[1]
        + offset_[1]
    )
    new_pos[0] = (map_res * downsampling_factor) * (pos[0]) + origin_[0] + offset_[0]

    return new_pos


def visualize_transform(file_path, points: np.ndarray):
    map_data = load_map_from_file(file_path)

    transformed_points = []
    for point in points:
        transformed_points.append(transform(point))
        print(transformed_points[-1])
    transformed_points = np.array(transformed_points)
    plt.figure()
    # return
    # Create a figure and axis for plotting
    # fig, ax = plt.subplots()

    # Display the map data
    plt.imshow(map_data, cmap="gray", origin="lower")

    # Add labels and title
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Occupancy Grid Map")

    # Plot the line
    plt.plot(
        transformed_points[:, 0],
        transformed_points[:, 1],
        color="red",
        label="Real points",
    )
    plt.scatter(
        transformed_points[:, 0],
        transformed_points[:, 1],
        color="red",
        label="Real points",
    )

    # Add a legend
    plt.legend()

    # Show the plot
    # plt.gca().invert_yaxis()
    plt.gca().invert_xaxis()
    # plt.show()


def visualize_transform_error(map_points):
    transformed_points = []
    un_transformed_points = []
    for point in map_points:
        transformed_points.append(transform(point))
        # print(transformed_points[-1])
        un_transformed_points.append(untransform(transformed_points[-1]))
    transformed_points = np.array(transformed_points)
    un_transformed_points = np.array(un_transformed_points)

    print(np.mean(np.linalg.norm(map_points - un_transformed_points, axis=1)))
    plt.figure(2)
    plt.plot(
        un_transformed_points[:, 0], un_transformed_points[:, 1], label="Transformed"
    )
    plt.plot(map_points[:, 0], map_points[:, 1], label="Original")
    plt.legend()
    plt.gca().invert_yaxis()
    plt.gca().invert_xaxis()
    plt.show()


map_points = np.array(
    [
        [-1, -1.0],
        [-1.0, -2.0],
        [-1.0, -2.8],
        [-2.0, -2.8],
        [-1.0, -3.0],
        [-1.0, -4.0],
        [-0.8, -5.0],
        [-1.0, -5.3],
        [-2.2, -6.0],
        [-2.2, -5.3],
        [0.08, -5.3],
        [1.15, -5.3],
        [1.15, -6.70],
        [1.65, -6.00],
        [1.77, -4.02],
        [1.80, -1.58],
        [1.54, -0.367],
        [0.64, -0.488],
        [-0.109, -0.517],
        [-1.0, -0.517],
        [-1.2, -0.5],
        [-1.5, 0.5],
        [-2.0, 1.0],
        [-2.0, 2.0],
    ]
)

visualize_transform(
    file_path="../maps/downsampled-map/Oct-16/Oct-16-downsampled.map",
    points=map_points,
)

visualize_transform_error(map_points)

# Specify the path to the map file
# file_path = "../maps/svd_demo-parsed-map.txt"

# Visualize the map
# visualize_map(file_path)
