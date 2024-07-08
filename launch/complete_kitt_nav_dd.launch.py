
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    robot_id = LaunchConfiguration('robot_id', default='kitt_nav_dd_01')
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='3.0')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='-2.0')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.0')
    # Setup project paths
    pkg_project_bringup_sim = get_package_share_directory('icai_crl_bringup_sim')
    pkg_project_bringup = get_package_share_directory('icai_crl_bringup')

    # Launch gazebo simulation and bridge
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup_sim, 'launch', 'yaml_simulation_launcher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'sim_diff_drive.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'robot_id': robot_id,
                          'initial_pose_x': initial_pose_x,
                          'initial_pose_y': initial_pose_y,
                          'initial_pose_yaw': initial_pose_yaw}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'rviz2_kitt.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'robot_id': robot_id,}.items(),
    )

    return LaunchDescription([
        gazebo_simulation,
        nav_launch,
        rviz
    ])