import numpy as np
import cv2

# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import yaml


class MapDownSampler:
    def __init__(self, map_file_path: str, map_name: str, robot_footprint: float):
        self.robot_footprint = robot_footprint  # meters default -> robot size

        if map_file_path[-1] != "/":
            map_file_path += "/"

        self.metadata_file_path = map_file_path + map_name + ".yaml"
        self.map_file_path = map_file_path + map_name + ".pgm"
        self.downsampled_map_file_path = map_file_path + map_name + "-downsampled.map"
        self.downsampled_metadata_file_path = (
            map_file_path + map_name + "-downsampled.yaml"
        )

        self.original_map_metadata = self.load_metadata_from_file(
            self.metadata_file_path
        )
        self.original_map = self.load_map_from_file(self.map_file_path)

        self.downsampled_map = None
        self.downsampling_scale = None
        self.translation = [0.0, 0.0]

    def load_map_from_file(self, file_path):
        map_data = None
        try:
            map_data = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
            print("\033[92m" + "Map file loaded successfully." + "\033[0m")
        except FileNotFoundError:
            print(f"Error: File '{file_path}' not found.")
        except Exception as e:
            print(f"An error occurred while loading the PGM file: {e}")
        map_data = 255 - map_data

        # self.visualize_map(map_data)

        return map_data

    def load_metadata_from_file(self, file_path):
        # Load meta.yaml
        data = None
        try:
            with open(file_path, "r") as yaml_file:
                data = yaml.safe_load(yaml_file)
            print("\033[92m" + "Metadata file loaded successfully." + "\033[0m")
        except FileNotFoundError:
            print(f"Error: File '{file_path}' not found." + "\033[0m")
        except Exception as e:
            print(f"An error occurred while loading the YAML file: {e}")
        return data

        # self.save_cbs_map(
        #     self.downsampled_map, self.downsampled_map_file_path
        # )

    def downsample_map(self, map, metadata, robot_cell_size=0.5, invert=False):
        print("Downsampling ...")
        downsampled_map = None
        downsampling_scale = None
        try:
            downsampling_scale = np.ceil(
                robot_cell_size / float(metadata["resolution"])
            )

            # do non maximal supression on thresh image with the kernel size
            map = cv2.dilate(
                map,
                np.ones((int(downsampling_scale), int(downsampling_scale))),
                iterations=1,
            )

            downsampled_map = cv2.resize(
                map,
                None,
                fx=1.0 / downsampling_scale,
                fy=1.0 / downsampling_scale,
                interpolation=cv2.INTER_NEAREST,
            )

            if invert:
                downsampled_map = 255 - downsampled_map
            print("\033[92m" + "Downsampling complete." + "\033[0m")
        except Exception as e:
            print(f"An error occurred while downsampling the map: {e}")

        return downsampled_map, downsampling_scale

    def run_downsampling(self, invert=False):
        self.downsampled_map, self.downsampling_scale = self.downsample_map(
            self.original_map, self.original_map_metadata, self.robot_footprint, invert
        )

    def visualize_map(self, map):
        fig, ax = plt.subplots()
        print("showing image")
        # Display the map data

        ax.imshow(np.flipud(map), cmap="gray", origin="lower")

        # Add labels and title
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("Occupancy Grid Map")

        # Show the plot
        plt.show()

    def save_cbs_map(self, map_data, map_file_path, metadata_file_path):
        # ------------------ CBS MAP FORMAT ------------------
        print(
            "\033[92m"
            + "writing downsampled map to file:"
            + metadata_file_path
            + "\033[0m"
        )

        with open(metadata_file_path, "w") as yaml_file:
            data = {
                "downsampling_scale": float(self.downsampling_scale),
                "original_resolution": float(self.original_map_metadata["resolution"]),
                "new_resolution": float(self.original_map_metadata["resolution"])
                * float(self.downsampling_scale),
                "translation": self.translation,
            }
            yaml.dump(data, yaml_file)

        print(
            "\033[92m" + "writing downsampled map to file:" + map_file_path + "\033[0m"
        )

        with open(map_file_path, "w") as f:
            f.write("type octile\n")
            f.write("height " + str(map_data.shape[0]) + "\n")
            f.write("width " + str(map_data.shape[1]) + "\n")
            f.write("map\n")
            # map_data = 255 - map_data
            map_data = map_data.astype(np.uint8)
            # map_data = map_data.T
            for row in map_data:
                for col in row:
                    if col > 200:
                        f.write(".")
                    else:
                        f.write("@")
                f.write("\n")


if __name__ == "__main__":
    map_downsampler = MapDownSampler(
        map_file_path="/home/vineet/mfi_maps/Sept8",
        map_name="croped_map",
        # map_file_path="/home/vineet/mfi_maps/svd_demo_final",
        # map_name="svd_demo",
        robot_footprint=0.6,  # 0.5 meters,
    )

    map_downsampler.run_downsampling(invert=True)
    map_downsampler.save_cbs_map(
        map_downsampler.downsampled_map,
        map_downsampler.downsampled_map_file_path,
        map_downsampler.downsampled_metadata_file_path,
    )
    map_downsampler.visualize_map(map_downsampler.downsampled_map)


# # Specify the path to the map file
# map_file_path = "../maps/workshop0-parsed-map.txt"
# file_path = "../maps/svd_demo-parsed-map.txt"
# downsample_map_file_path = "../maps/workshop0-downsampled-map.txt"
