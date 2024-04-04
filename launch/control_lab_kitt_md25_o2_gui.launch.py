
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('icai_crl_bringup')
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    world_sdf_path = os.path.join(pkg_project_gazebo, 'worlds', 'control_laboratory.sdf')
    config_gui_path = os.path.join(pkg_project_bringup, 'config', 'gazebo_gui.config')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf_path + ' -v --gui-config ' + config_gui_path}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'kitt_md25_bridge_o2.yaml'),
        }],
        output='screen'
    )

    # Spawn the car model in the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'kitt_md25_o2',
                   '-x', '3',
                   '-y', '-3',
                   '-z', '0.1',
                   '-Y', '3.1416',
                   '-file', os.path.join(pkg_project_gazebo, 'models', 'md25_driver', 'kitt_md25_o2')],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        spawn_entity
    ])