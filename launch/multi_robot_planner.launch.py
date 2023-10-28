import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# MY_NO_ROBOTS = os.environ["Number_of_Robots"]
# MY_NEO_ENVIRONMENT = os.environ.get("MAP_NAME", "mfi")


def generate_launch_description():
    # Get the path to the ROS2 package
    my_package_share_directory = get_package_share_directory("ddg_multi_robot_planner")
    # Create the ROS2 launch description
    ld = LaunchDescription()
    use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
        description="Simulation or Real)",
    )
    num_robots = DeclareLaunchArgument(
        "num_robots",
        default_value="2",
        description="Number of robots",
    )
    use_inbuilt_waypoint_follower = DeclareLaunchArgument(
        "use_inbuilt_waypoint_follower",
        default_value="False",
        description="Creates subscribers and publisher for all topics requried for waypoint follower for N robots",
    )
    # Downsampled map parameters
    downsampled_map_file_path = DeclareLaunchArgument(
        "downsampled_map_file_path",
        default_value=os.path.join(
            my_package_share_directory,
            "maps/downsampled-map/res20/svd_demo-downsampled.map",
        ),
        description="Path to the map file",
    )
    downsampling_factor = DeclareLaunchArgument(
        "downsampling_factor",
        default_value="20.0",
        description="downsampling scale from the original map",
    )
    downsampling_faorignal_map_resolutionctor = DeclareLaunchArgument(
        "downsampling_faorignal_map_resolutionctor",
        default_value="0.05",
        description="resolution of the original map",
    )
    original_origin = DeclareLaunchArgument(
        "original_origin",
        default_value="['-3.96', '-3.26']",
        description="origin of the original map",
    )
    offset = DeclareLaunchArgument(
        "offset",
        default_value="['0.2', '0.7']",
        description="Offest to be added to the downsmpled map such that the tranform results in waypoints at the center of the cells",
    )
    original_map_size = DeclareLaunchArgument(
        "original_map_size",
        default_value="['443', '149']",
        description="size of the original map",
    )

    ld.add_action(use_sim)
    ld.add_action(num_robots)
    ld.add_action(use_inbuilt_waypoint_follower)

    # Downsampled map parameters
    ld.add_action(downsampled_map_file_path)
    ld.add_action(downsampling_factor)
    ld.add_action(downsampling_faorignal_map_resolutionctor)
    ld.add_action(original_origin)
    ld.add_action(offset)
    ld.add_action(original_map_size)

    ddg_multi_robot_planner_node = Node(
        package="ddg_multi_robot_planner",
        executable="ddg_multi_robot_planner_node",
        output="screen",
        parameters=[
            {"use_sim": LaunchConfiguration("use_sim")},
            {"num_robots": LaunchConfiguration("num_robots")},
            {
                "use_inbuilt_waypoint_follower": LaunchConfiguration(
                    "use_inbuilt_waypoint_follower"
                )
            },
            {
                "downsampled_map_file_path": LaunchConfiguration(
                    "downsampled_map_file_path"
                )
            },
            {"downsampling_factor": LaunchConfiguration("downsampling_factor")},
            {
                "downsampling_faorignal_map_resolutionctor": LaunchConfiguration(
                    "downsampling_faorignal_map_resolutionctor"
                )
            },
            {"original_origin": LaunchConfiguration("original_origin")},
            {"offset": LaunchConfiguration("offset")},
            {"original_map_size": LaunchConfiguration("original_map_size")},
        ],
    )

    # Add the node to the launch description
    ld.add_action(ddg_multi_robot_planner_node)

    return ld
