
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup_sim = get_package_share_directory('icai_crl_bringup_sim')
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_hardware_bridge = get_package_share_directory('hardware_bridge')

    # Setup to launch the simulator and Gazebo world
    world_sdf_path = os.path.join(pkg_project_gazebo, 'worlds', 'control_laboratory.sdf')
    control_laboratory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf_path + ' -r -s'}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge_node',
        parameters=[
            {'config_file': os.path.join(pkg_project_bringup_sim, 'config', 'kitt_md25_bridge.yaml')},
            {'expand_gz_topic_names': True}
        ],
        namespace=['/model/kitt'],
        output='screen'
    )

    # Spawn the car model in the Gazebo world
    kitt_md25_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-x', '0.5',
                   '-y', '-5.4',
                   '-z', '0.01',
                   '-Y', '0.1',
                   '-file', os.path.join(pkg_project_gazebo, 'models', 'md25_driver', 'kitt_md25')],
        output='screen'
    )

    # Launch the hardware bridge to communicate with the Raspberry Pi
    hardware_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hardware_bridge, 'launch', 'hardware_bridge.launch.py')),
        launch_arguments={'ip_address': '192.168.2.123','send_period_ms': '5','namespace': '/model/kitt'}.items(),
    )

    return LaunchDescription([
        bridge,
        control_laboratory,
        kitt_md25_robot,
        hardware_bridge
    ])