
import os

from jinja2 import Environment, FileSystemLoader
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_model_files(context: LaunchContext, model_name: str):
    # Setup project paths
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')

    model_name_str=context.perform_substitution(model_name)
    # Setup model directory
    model_dir = os.path.join(pkg_project_gazebo, "models", "diff_drive", model_name_str)
    if not os.path.exists(model_dir):
        # Create the model directory
        os.makedirs(model_dir)

        # Create a Jinja2 environment with the template directory
        env = Environment(loader=FileSystemLoader(os.path.join(
            pkg_project_gazebo, 'models', 'diff_drive', 'kitt_dd_template')))

        # Render the templates and write the output files
        for template_name, output_name in [('kitt_dd_template.sdf.jinja', f'{model_name_str}.sdf'),
                                        ('model.config.jinja', 'model.config')]:
            template = env.get_template(template_name)
            output = template.render(model_name=model_name_str)
            with open(os.path.join(model_dir, output_name), 'w') as f:
                f.write(output)
        print(f"Generated model files for {model_name_str}")
    else:
        print(f"Model files for {model_name_str} already exist, skipping generation")
    return

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('icai_crl_bringup')
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')


    # kitt name argument
    kitt_name = DeclareLaunchArgument(
        'kitt_name',
        default_value='kitt_dd_01',
        description='Name of the KITT model in Gazebo'
    )
    kitt_name_arg = LaunchConfiguration('kitt_name')

    # Labeled model creator 
    create_files = OpaqueFunction(function=generate_model_files, args=[kitt_name_arg])


    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='kitt_dd_bridge',
        parameters=[
            {'config_file': os.path.join(pkg_project_bringup, 'config', 'kitt_dd_bridge.yaml')},
            {'expand_gz_topic_names': True}
        ],
        namespace=['/model/', kitt_name_arg],
        output='screen'
    )

    initial_x = DeclareLaunchArgument(
        'initial_x',
        default_value='3',
        description='X model-spawn coordinate in Gazebo'
    )
    initial_y = DeclareLaunchArgument(
        'initial_y',
        default_value='-3',
        description='Y model-spawn coordinate in Gazebo'
    )
    initial_z = DeclareLaunchArgument(
        'initial_z',
        default_value='0.01',
        description='Z model-spawn coordinate in Gazebo'
    )
    initial_Y = DeclareLaunchArgument(
        'initial_Y',
        default_value='3.1416',
        description='Yaw model-spawn angle in Gazebo'
    )
    initial_x_arg = LaunchConfiguration('initial_x')
    initial_y_arg = LaunchConfiguration('initial_y')
    initial_z_arg = LaunchConfiguration('initial_z')
    initial_Y_arg = LaunchConfiguration('initial_Y')

    # Spawn the car model in the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        namespace= kitt_name_arg,
        arguments=['-x', initial_x_arg,
                   '-y', initial_y_arg,
                   '-z', initial_z_arg,
                   '-Y', initial_Y_arg,
                   '-file', PathJoinSubstitution([pkg_project_gazebo, 'models', 'diff_drive', kitt_name_arg])],
        output='screen'
    )

    return LaunchDescription([
        create_files,
        kitt_name,
        bridge,
        spawn_entity,
        initial_x,
        initial_y,
        initial_z,
        initial_Y
    ])