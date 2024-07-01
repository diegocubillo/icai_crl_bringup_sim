import os

from ament_index_python.packages import get_package_share_directory

from jinja2 import Environment, FileSystemLoader

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import yaml

N = 1


def load_yaml_file():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory("icai_crl_bringup")
    # Load the YAML file
    formatted_N = f"{N:03}"
    with open(os.path.join(pkg_project_bringup, "config", f"launch_{formatted_N}.yaml"), "r") as file:
        data = yaml.safe_load_all(file)
        loaded_data = list(data)
    print(f"Loading data from file launch_{formatted_N}.yaml")

    # First element is the world name
    world_name = loaded_data[0][0].get('world_name')
    print(f"World name: {world_name}")

    # Create a list of robot launch descriptions
    robots = []

    # Create a list of items
    items = []

    for element in loaded_data[0]:
        # Fill the robots list
        if "robot_name" in element:
            robots.append((element.get('robot_name'), element.get('driver_name'), element.get('navigation_stack'), element.get('initial_x'), element.get('initial_y'), element.get('initial_z'), element.get('initial_yaw')))
            print(f"Robot: {element.get('robot_name')}, driver: {element.get('driver_name')}, navigation stack: {element.get('navigation_stack')}, x: {element.get('initial_x')}, y: {element.get('initial_y')}, z: {element.get('initial_z')}, Y: {element.get('initial_yaw')}")
        # Fill the items list
        elif "item_name" in element:
            items.append((element.get('item_name'), element.get('initial_x'), element.get('initial_y'), element.get('initial_z'), element.get('initial_yaw')))
            print(f"Item: {element.get('item_name')}, x: {element.get('initial_x')}, y: {element.get('initial_y')}, z: {element.get('initial_z')}, Y: {element.get('initial_yaw')}")


    return world_name, robots, items



def generate_model_and_launcher(robot_model, driver, nav_stack, x, y, z, Y, index, ld):
    # Setup project paths
    pkg_project_bringup = get_package_share_directory("icai_crl_bringup")
    pkg_project_gazebo = get_package_share_directory('icai_crl_gazebo')


    # Setup model name
    if driver == 'diff_drive':
        driver_text = 'dd'
    elif driver == 'md25_driver':
        driver_text = 'md25'
    else:
        driver_text = 'dd'

    nav_text = ''
    if nav_stack:
        nav_text = '_nav'

    formatted_i = f"{index:02}"
    model_name = f'{robot_model}{nav_text}_{driver_text}'
    model_id = f'{model_name}_{formatted_i}'


    # Setup model directory
    model_dir = os.path.join(pkg_project_gazebo, "models", driver, model_id)
    if not os.path.exists(model_dir):
        # Create the model directory
        os.makedirs(model_dir)

        # Create a Jinja2 environment with the template directory
        env = Environment(loader=FileSystemLoader(os.path.join(
            pkg_project_gazebo, 'models', driver, f'{model_name}_template')))

        # Render the templates and write the output files
        for template_name, output_name in [(f'{model_name}_template.sdf.jinja', f'{model_id}.sdf'),
                                        ('model.config.jinja', 'model.config')]:
            template = env.get_template(template_name)
            output = template.render(model_name=model_id)
            with open(os.path.join(model_dir, output_name), 'w') as f:
                f.write(output)
        print(f"Generated model files for {model_id}")
    else:
        print(f"Model files for {model_id} already exist, skipping generation")


    # Launch the bridge and robot
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[
            {'config_file': os.path.join(pkg_project_bringup, 'config', f'{model_name}_bridge.yaml')},
            {'expand_gz_topic_names': True}
        ],
        namespace=['/model/', model_id],
        output='screen'
    )
    ld.add_action(bridge)

    robot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace= model_id,
        arguments=['-x', str(x),
                   '-y', str(y),
                   '-z', str(z),
                   '-Y', str(Y),
                   '-file', os.path.join(pkg_project_gazebo, 'models', driver, model_id)],
        output='screen'
    )
    ld.add_action(robot)

    # check if the robot has a navigation stack and is of type kitt
    if (nav_stack) and (robot_model == 'kitt'):
        static_transform_publisher_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_transform_broadcaster',
            namespace= model_id,
            arguments=['0.1075', '-0.03295', '0.145', '0', '0', '0', '1', f'{model_id}/car_body', f'{model_id}/nav_module/rplidar_a2m8'],
            output='screen',
        )
        ld.add_action(static_transform_publisher_node)
    return



def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("icai_crl_bringup")
    pkg_project_gazebo = get_package_share_directory("icai_crl_gazebo")
    pkg_project_description = get_package_share_directory('icai_crl_description')
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Setup to launch the simulator and Gazebo world
    world_name, robots, items = load_yaml_file()
    world_sdf_path = os.path.join(pkg_project_gazebo, "worlds", f"{world_name}.sdf")
    config_gui_path = os.path.join(pkg_project_bringup, "config", "gazebo_gui.config")
    
    ld = LaunchDescription()

    # Launch the simulator and Gazebo world
    control_laboratory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": world_sdf_path + " -v --gui-config " + config_gui_path
        }.items(),
    )
    ld.add_action(control_laboratory)

    # Generate a launch description for each robot
    i = 1
    for robot in robots:
        generate_model_and_launcher(*robot, i, ld)
        i += 1
    i = 1
    for item in items:
        item_name = item[0]
        item_id = f'{item_name}_{i}'
        node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace= item_id,
        arguments=['-name', item_id,
                   '-x', str(item[1]),
                   '-y', str(item[2]),
                   '-z', str(item[3]),
                   '-Y', str(item[4]),
                   '-file', os.path.join(pkg_project_description, 'models', 'environments', item_name)],
        output='screen'
    )
        i += 1
        ld.add_action(node)

    # Return the launch description
    return ld
