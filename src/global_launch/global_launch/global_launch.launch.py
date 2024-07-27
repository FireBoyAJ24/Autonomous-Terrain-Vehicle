import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Load the SDF file from the package
    package_name = "simulator"
    pkg_project_description = get_package_share_directory("description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Load the world file from the package
    with open(
        os.path.join(
            get_package_share_directory(package_name),
            "description",
            "main_car",
            "main_car.sdf",
        ),
        "r",
    ) as main_car_file:
        robot_desc = main_car_file.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [
                    get_package_share_directory(package_name),
                    "description",
                    "World",
                    "robotics_world.sdf",
                ]
            )
        }.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory(package_name),
                    "description",
                    "bridge",
                    "gz_ros_path.yaml",
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,
            bridge,
            robot_state_publisher,
        ]
    )
