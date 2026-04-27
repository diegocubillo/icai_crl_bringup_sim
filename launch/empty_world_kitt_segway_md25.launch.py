
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup_sim = get_package_share_directory('icai_crl_bringup_sim')
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')
    pkg_project_description = get_package_share_directory('icai_crl_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_hardware_bridge = get_package_share_directory('hardware_bridge')

    # Setup to launch the simulator and Gazebo world
    world_sdf_path = os.path.join(pkg_project_gazebo, 'worlds', 'empty_world.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf_path + ' -r -s'}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': os.path.join(pkg_project_bringup_sim, 'config', 'kitt_md25_bridge.yaml')},
            {'expand_gz_topic_names': True}
        ],
        namespace=['/model/kitt'],
        output='screen'
    )

    # Spawn the car model in the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-P', '-0.01',
                   '-Y', '0.1',
                   '-file', os.path.join(pkg_project_gazebo, 'models', 'md25_driver', 'kitt_segway_md25')],
        output='screen'
    )

    # Spawn the support wall for segway startup
    support_wall = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-x', '-0.085',
                   '-Y', '0.1',
                   '-file', os.path.join(pkg_project_description, 'models', 'environments', 'support_wall')],
        output='screen'
    )

    # Launch the hardware bridge to communicate with the Raspberry Pi
    hardware_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hardware_bridge, 'launch', 'hardware_bridge.launch.py')),
        launch_arguments={'ip_address': '192.168.2.123','send_period_ms': '5','namespace': '/model/kitt'}.items(),
    )

    virtual_buttons = Node(
        package='rqt_kitt_hmi',
        executable='rqt_kitt_hmi',
        namespace=['/model/kitt'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        spawn_entity,
        support_wall,
        hardware_bridge,
        # virtual_buttons
    ])