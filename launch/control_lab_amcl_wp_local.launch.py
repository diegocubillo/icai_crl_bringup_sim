
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('icai_crl_bringup')
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    base_frame = LaunchConfiguration('base_frame', default='kitt/car_body')
    odom_frame = LaunchConfiguration('odom_frame', default='kitt/odom')
    map_frame = LaunchConfiguration('map_frame', default='control_laboratory')

    # Setup to launch the simulator and Gazebo world
    world_sdf_path = os.path.join(pkg_project_gazebo, 'worlds', 'control_laboratory.sdf')
    map_yaml_path = os.path.join(pkg_project_bringup, 'map', 'map.yaml')
    config_gui_path = os.path.join(pkg_project_bringup, 'config', 'gazebo_gui.config')
    # localization_launch_dir = os.path.join(pkg_nav2_bringup, 'launch')
    # localization_params_file = os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf_path + ' -r --gui-config ' + config_gui_path}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_file': os.path.join(pkg_project_bringup, 'config', 'kitt_nav_md25_bridge.yaml')},
            {'expand_gz_topic_names': True}
        ],
        namespace=['/model/kitt'],
        output='screen'
    )

    # Spawn the car model in the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-x', '3',
                   '-y', '-3',
                   '-z', '0.01',
                   '-Y', '3.1416',
                   '-file', os.path.join(pkg_project_gazebo, 'models', 'md25_driver', 'kitt_nav_md25')],
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
                   '--frame-id', base_frame,
                   '--child-frame-id', 'kitt/nav_module/rplidar_a2m8'],
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=['/model/kitt'],
        output='screen',
    )

    # second_static_transform_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='car_body_transform_broadcaster',
    #     arguments=['--x', '0',
    #                '--y', '0',
    #                '--z', '0.05',
    #                '--qx', '0',
    #                '--qy', '0',
    #                '--qz', '0',
    #                '--qw', '1',
    #                '--frame-id', 'kitt_dd',
    #                '--child-frame-id', 'kitt_dd/car_body'],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    # )

    # odom_static_transform_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_transform_broadcaster',
    #     arguments=['--x', '3',
    #                '--y', '-3',
    #                '--z', '0.01',
    #                '--qx', '0',
    #                '--qy', '0',
    #                '--qz', '1',
    #                '--qw', '0',
    #                '--frame-id', 'control_laboratory_with_car',
    #                '--child-frame-id', 'odom'],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    # )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # slam_online_async = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'odom_frame': 'odom'},
    #                 {'base_frame': 'kitt_dd'},
    #                 {'map_frame': 'control_laboratory_with_car'},
    #                 {'map_file_name': map_yaml_path},
    #                 {'map_start_pose': [3.0, -3.0, 3.1416]}, # x, y, theta,
    #                 {'max_laser_range': 8.0},
    #                 {'mode': 'localization'},
    #                 {'use_scan_matching': False}
    #                 ],
    #     output='screen',
    # )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=['/model/kitt'],
        parameters=[{'use_sim_time': use_sim_time},
                    {'set_initial_pose': True},
                    # {'initial_pose': [3.0, -3.0, 0.01]}, #, 3.1416]}, # x, y, z, theta
                    {'initial_pose.x': 3.0},
                    {'initial_pose.y': -3.0},
                    {'initial_pose.z': 0.01},
                    {'initial_pose.yaw': 3.1416},
                    {'odom_frame_id': odom_frame},
                    {'base_frame_id': base_frame},
                    {'global_frame_id': map_frame},
                    {'first_map_only': True},
                    {'scan_topic': 'rplidar_a2m8/scan'},],
        output='screen',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        namespace=['/model/kitt'],
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_path},
                    {'frame_id': map_frame}],
        output='screen',
    )

    # lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'autostart': True},
    #                 {'node_names': ['map_server']}]
    # )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        namespace=['/model/kitt'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['amcl', 'map_server']}]
    )

    odom_to_tf_node = Node(
            package='odom_to_tf_ros2',
            executable='odom_to_tf',
            namespace='/model/kitt',
            parameters=[{'frame_id': odom_frame},
                        {'child_frame_id': base_frame},
                        {'odom_topic': 'car_odom'}],
            output='screen')

    # localization = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(localization_launch_dir,
    #                                                    'localization_launch.py')),
    #         launch_arguments={'namespace': '/model/kitt',
    #                           'map': map_yaml_path,
    #                           'use_sim_time': use_sim_time,
    #                           'autostart': 'true',
    #                           'params_file': localization_params_file,
    #                           'use_composition': 'False',
    #                           'use_respawn': 'False'}.items())
    
    # Launch the ROS2 node with the specified namespace, package, and executable
    kitt_node = Node(
        package='car_control_system',
        executable='CAR_CONTROL_SYSTEM',
        namespace=['/model/kitt'],
        output='screen'
    )

    tf2_to_pose_node = Node(
            package='tcp_ros_bridge',
            executable='tf2_to_pose',
            namespace='/model/kitt',
            parameters=[{'target_frame': base_frame},
                        {'source_frame': map_frame}],
            output='screen')


    return LaunchDescription([
        gz_sim,
        bridge,
        spawn_entity,
        static_transform_publisher_node,
        # second_static_transform_publisher_node,
        # odom_static_transform_publisher_node,
        rviz2,
        # slam_online_async,
        amcl_node,
        map_server_node,
        lifecycle_manager,
        # localization,
        kitt_node,
        odom_to_tf_node,
        tf2_to_pose_node
    ])