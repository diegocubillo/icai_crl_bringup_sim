
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
    model_dir = os.path.join(pkg_project_gazebo, "models", "md25_driver", model_name_str)
    if not os.path.exists(model_dir):
        # Create the model directory
        os.makedirs(model_dir)

        # Create a Jinja2 environment with the template directory
        env = Environment(loader=FileSystemLoader(os.path.join(
            pkg_project_gazebo, 'models', 'md25_driver', 'kitt_md25_template')))

        # Render the templates and write the output files
        for template_name, output_name in [('kitt_md25_template.sdf.jinja', f'{model_name_str}.sdf'),
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
        default_value='kitt_md25',
        description='Name of the KITT model in Gazebo'
    )
    kitt_name_arg = LaunchConfiguration('kitt_name')

    # Labeled model creator 
    create_files = OpaqueFunction(function=generate_model_files, args=[kitt_name_arg])


    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='kitt_md25_bridge',
        parameters=[
            {'config_file': os.path.join(pkg_project_bringup, 'config', 'kitt_md25_bridge.yaml')},
            {'expand_gz_topic_names': True}
        ],
        namespace=['/model/', kitt_name_arg],
        output='screen'
    )

    # Spawn the car model in the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        namespace= kitt_name_arg,
        arguments=['-x', '3',
                   '-y', '-3',
                   '-z', '0.1',
                   '-Y', '3.1416',
                   '-file', PathJoinSubstitution([pkg_project_gazebo, 'models', 'md25_driver', kitt_name_arg])], #os.path.join(pkg_project_gazebo, 'models', 'md25_driver', 'kitt_md25')],
        output='screen'
    )

    return LaunchDescription([
        create_files,
        kitt_name,
        bridge,
        spawn_entity
    ])