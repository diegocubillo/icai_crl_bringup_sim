
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
    pkg_horizontal_image_view = get_package_share_directory('horizontal_image_couple')

    # Setup to launch the simulator and Gazebo world
    world_sdf_path = os.path.join(pkg_project_gazebo, 'worlds', 'control_laboratory.sdf')
    config_gui_path = os.path.join(pkg_project_bringup, 'config', 'gazebo_gui.config')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf_path + ' -r --gui-config ' + config_gui_path}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'kitt_fpv_bridge.yaml'),
        }],
        output='screen'
    )

    # Spawn the car model in the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'kitt_dd',
                   '-x', '3',
                   '-y', '-3',
                   '-z', '0.1',
                   '-Y', '3.1416',
                   '-file', os.path.join(pkg_project_gazebo, 'models', 'diff_drive', 'kitt_fpv_dd')],
        output='screen'
    )

    horizontal_fpv_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_horizontal_image_view + '/launch/kitt_fpv_launch.py')
    )

    # Open rqt_image_view to visualize the fpv image
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/kitt/coupled_fpv_camera'],
        output='screen'
    )

    # Joy node for controlling the car
    joy = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    # Teleop node for controlling the car
    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'FlySky_FS-i6S.config.yaml')],
        remappings=[('/cmd_vel', '/kitt/cmd_vel')],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        spawn_entity,
        horizontal_fpv_camera,
        rqt_image_view,
        joy,
        teleop
    ])