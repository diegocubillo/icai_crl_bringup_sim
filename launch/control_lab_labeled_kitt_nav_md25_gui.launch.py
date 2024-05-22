
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
    pkg_project_bringup = get_package_share_directory('icai_crl_bringup')
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    world_sdf_path = os.path.join(pkg_project_gazebo, 'worlds', 'control_laboratory.sdf')
    config_gui_path = os.path.join(pkg_project_bringup, 'config', 'gazebo_gui.config')
    kitt_md25_launch_path = os.path.join(pkg_project_bringup, 'launch', 'labeled_kitt_nav_md25.py')

    kitt_name = DeclareLaunchArgument(
        'kitt_name',
        default_value='kitt_nav_md25_01',
        description='Name of the KITT model in Gazebo'
    )
    kitt_name_arg = LaunchConfiguration('kitt_name')

    control_laboratory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf_path + ' -v --gui-config ' + config_gui_path}.items(),
    )


    # Bridge ROS topics and Gazebo messages for establishing communication
    kitt_md25_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([kitt_md25_launch_path]),
        launch_arguments={'kitt_name': kitt_name_arg}.items(),
    )

    return LaunchDescription([
        kitt_name,
        control_laboratory,
        kitt_md25_robot
    ])