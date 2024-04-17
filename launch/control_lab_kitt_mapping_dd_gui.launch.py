
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Configure ROS nodes for launch
    # Once all the map is scanned, run the following command to save the map:
    # ros2 run nav2_map_server map_saver_cli -f <path_to_save_map or map_name>
    # for example: ros2 run nav2_map_server map_saver_cli -f my_map

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('icai_crl_bringup')
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Setup to launch the simulator and Gazebo world
    world_sdf_path = os.path.join(pkg_project_gazebo, 'worlds', 'control_laboratory.sdf')
    # map_yaml_path = os.path.join(pkg_project_bringup, 'map', 'map.yaml')
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
        parameters=[{'use_sim_time': use_sim_time},
                    {'config_file': os.path.join(pkg_project_bringup, 'config', 'kitt_mapping_bridge.yaml')}],
        output='screen'
    )

    # Spawn the car model in the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'kitt_dd',
                   '-x', '3',
                   '-y', '-3',
                   '-z', '0.01',
                   '-Y', '3.1416',
                   '-file', os.path.join(pkg_project_gazebo, 'models', 'diff_drive', 'kitt_nav_dd')],
        output='screen'
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_transform_broadcaster',
        arguments=['--x', '0.1075',
                   '--y', '-0.03295',
                   '--z', '0.145',
                   '--qx', '0',
                   '--qy', '0',
                   '--qz', '0',
                   '--qw', '1',
                   '--frame-id', 'kitt_dd/car_body',
                   '--child-frame-id', 'kitt_dd/nav_module/rplidar_a2m8'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    second_static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='car_body_transform_broadcaster',
        arguments=['--x', '0',
                   '--y', '0',
                   '--z', '0.05',
                   '--qx', '0',
                   '--qy', '0',
                   '--qz', '0',
                   '--qw', '1',
                   '--frame-id', 'kitt_dd',
                   '--child-frame-id', 'kitt_dd/car_body'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    odom_static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_transform_broadcaster',
        arguments=['--x', '3',
                   '--y', '-3',
                   '--z', '0.01',
                   '--qx', '0',
                   '--qy', '0',
                   '--qz', '1',
                   '--qw', '0',
                   '--frame-id', 'control_laboratory_with_car',
                   '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    slam_online_async = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{'use_sim_time': use_sim_time},
                    {'odom_frame': 'odom'},
                    {'base_frame': 'kitt_dd'},
                    {'map_frame': 'control_laboratory_with_car'},
                    # {'map_file_name': map_yaml_path},
                    {'map_start_pose': [3.0, -3.0, 3.1416]}, # x, y, theta,
                    {'max_laser_range': 8.0},
                    {'mode': 'mapping'},
                    {'use_scan_matching': False}
                    ],
        output='screen',
    )


    return LaunchDescription([
        gz_sim,
        bridge,
        spawn_entity,
        static_transform_publisher_node,
        second_static_transform_publisher_node,
        odom_static_transform_publisher_node,
        rviz2,
        slam_online_async
    ])