
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node



# Launches the laboratory world and kitt_nav model in Gazebo, gz_ros_bridge, and the joy and teleop nodes
# configured to control kitt_nav via twist messages with FlySky FS-i6s controller.

# Left stick controls linear velocity and right stick controls angular velocity.
# The left 3-way switch enables the control of the car in its lower position.
# The right 2-way switch increases the velocity references of the car.

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('icai_crl_bringup')

    # Setup to launch the simulator and Gazebo world, and ros_gz_bridge
    gazebo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                pkg_project_bringup + '/launch/control_lab_kitt_nav_dd_gui.launch.py')
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
        gazebo_world_launch,
        joy,
        teleop
    ])