# icai_crl_bringup_sim

This package provides the launch and configuration files for the robot simulations at ICAI's Control and Robotics Laboratory.

This package integrates the simulation models from `icai_crl_description` and the Gazebo plugins from `icai_crl_gazebo` to provide a complete, ready-to-run simulation environment.

## Installation

### Prerequisites

- **ROS 2:** Humble Hawksbill
- **Gazebo:** Fortress
- **ROS-Gazebo Bridge:** `ros_gz`

Follow the official installation guides:
1.  [Install ROS 2 Humble](http://docs.ros.org/en/humble/Installation.html)
2.  [Install Gazebo Fortress and ros_gz](https://gazebosim.org/docs/fortress/ros_installation)

### Dependencies

This package requires the following repositories to be present in the same ROS 2 workspace:

-   **[icai_crl_description](https://github.com/diegocubillo/icai_crl_description)**: Contains the robot and environment description files (SDF).
-   **[icai_crl_gazebo](https://github.com/diegocubillo/icai_crl_gazebo)**: Contains custom Gazebo plugins for simulating actuators and sensors.

### Building

Clone this repository and its dependencies into the `src/` directory of your ROS 2 workspace, then build the packages:
```bash
colcon build
```

## Launch Files

This package contains several launch files to start different simulation scenarios.

### Parametrizable Launcher

-   `yaml_simulation_launcher.launch.py`: This is the core launcher of the package. It is a versatile script that spawns a complete simulation environment—including the world, robots, and other items—from a single YAML configuration file. This allows for easy definition and modification of complex scenarios. The YAML file can be selected through the ROS 2 integer launch parameter `scenario_number`.

### Pre-configured Scenarios

The following launch files provide ready-to-use examples for specific robot configurations and environments:

-   `control_lab_kitt_dd_gui.launch.py`: Launches the `kitt` robot with a `diff_drive` controller in the `control_laboratory` world.
-   `control_lab_kitt_md25_gui.launch.py`: Launches the `kitt` robot with the custom `MD25` motor controller plugin in the `control_laboratory` world.
-   `control_lab_kitt_segway_md25_gui.launch.py`: Launches the `kitt_segway` robot with the custom `MD25` motor controller plugin in the `control_laboratory` world.
-   `ramp_circuit_kitt_dd_gui.launch.py`: Launches the `kitt` robot with a `diff_drive` controller in the `ramp_circuit` competition world.
-   `ramp_circuit_kitt_md25_gui.launch.py`: Launches the `kitt` robot with the `MD25` motor controller in the `ramp_circuit` world.
-   `ramp_circuit_kitt_segway_md25_gui.launch.py`: Launches the `kitt_segway` robot with the `MD25` motor controller in the `ramp_circuit` world.

To run a launch file, source your workspace and execute the following command, replacing `<launch_file_name>` with your desired file:
```bash
ros2 launch icai_crl_bringup_sim <launch_file_name>
```

## Configuration Files

This package includes all the necessary configuration files to define simulation scenarios and communication bridges.

### Simulation Scenario Files

-   `launch_001.yaml`, `launch_002.yaml`, `launch_003.yaml`, `launch_004.yaml`, `launch_005.yaml`: These YAML files define complete simulation scenarios for the `yaml_simulation_launcher.launch.py`. They specify which world to load, which robots and items to spawn, and their initial positions and orientations.

### ROS <=> Gazebo Bridges

These files configure the `ros_gz_bridge` to translate messages between ROS 2 topics and Gazebo Transport topics.

-   `kitt_dd_bridge.yaml`: Bridges topics for the `kitt` robot with `diff_drive`, including velocity commands, odometry, IMU data, and TF transforms.
-   `kitt_md25_bridge.yaml`: Bridges topics for the `kitt` robot with the `MD25` controller, including motor voltage commands, odometry, IMU data, and TF transforms.
<!-- -   `kitt_nav_dd_bridge.yaml` / `kitt_nav_md25_bridge.yaml`: Extensions of the above bridges that also include topics for a 360º LiDAR sensor for navigation tasks. -->

### Gazebo GUI Configuration

-   `gazebo_gui.config`: Default GUI layout for the Gazebo window.
-   `gazebo_debug_gui.config`: A GUI layout with additional plugins and widgets useful for debugging and analyzing the simulation.
