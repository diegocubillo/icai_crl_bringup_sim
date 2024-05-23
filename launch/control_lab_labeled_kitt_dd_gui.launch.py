import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

N = 1


def load_yaml_file():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory("icai_crl_bringup")
    # Load the YAML file
    formatted_N = f"{N:03}"
    with open(os.path.join(pkg_project_bringup, "config", f"launch_{formatted_N}.yaml"), "r") as file:
        data = yaml.safe_load_all(file)
        loaded_data = list(data)
    print(f"Loading data from file launch_{formatted_N}.yaml")

    # First element is the world name
    world_name = loaded_data[0][0].get('world_name')

    # Fill the list of robots
    print(f"World name: {world_name}")

    # # Count the number of robots
    # num_robots = len(loaded_data[1])
    # print(f"Number of robots: {num_robots}")

    # Create a list of robot launch descriptions
    robots = []
    for robot in loaded_data[1]:
        robots.append((robot.get('robot_name'), robot.get('initial_x'), robot.get('initial_y'), robot.get('initial_z'), robot.get('initial_yaw')))
        print(f"Robot: {robot.get('robot_name')}, x: {robot.get('initial_x')}, y: {robot.get('initial_y')}, z: {robot.get('initial_z')}, Y: {robot.get('initial_yaw')}")
    
    return robots, world_name

def generate_robot_launch_description(name, x, y, z, Y):
    pkg_project_bringup = get_package_share_directory("icai_crl_bringup")
    kitt_dd_launch_path = os.path.join(
        pkg_project_bringup, "launch", "labeled_kitt_dd.py"
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([kitt_dd_launch_path]),
        launch_arguments={
            "kitt_name": name,
            "initial_x": str(x),
            "initial_y": str(y),
            "initial_z": str(z),
            "initial_Y": str(Y),
        }.items(),
    )


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("icai_crl_bringup")
    pkg_project_gazebo = get_package_share_directory("icai_crl_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Setup to launch the simulator and Gazebo world
    robots, world_name = load_yaml_file()
    world_sdf_path = os.path.join(pkg_project_gazebo, "worlds", f"{world_name}.sdf")
    config_gui_path = os.path.join(pkg_project_bringup, "config", "gazebo_gui.config")

    # Launch the simulator and Gazebo world
    control_laboratory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": world_sdf_path + " -v --gui-config " + config_gui_path
        }.items(),
    )

    # Generate a launch description for each robot and launch them
    robot_launch_descriptions = [generate_robot_launch_description(*robot) for robot in robots]

    return LaunchDescription(
        [
            control_laboratory,
            *robot_launch_descriptions
        ]
    )
