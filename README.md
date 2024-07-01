# icai_crl_bringup_sim

Launch and configuration files for robot simulation of ICAI's Control and Robotics Laboratory.

## Installation

### Dependencies
To use this package, you need to have ROS 2 humble, Gazebo Fortress and ros_gz installed on your linux system (binaries are available for Ubuntu 22.04). If you haven't installed them yet, follow these steps:

1. Install ROS by following the instructions at [ROS Installation Guide](http://docs.ros.org/en/humble/Installation.html).
2. Install Gazebo and ros_gz by following the instructions at [Gazebo with ROS Installation Guide](https://gazebosim.org/docs/fortress/ros_installation).

### Other Repositories

This package makes use of the packages contained in the following repositories:

- [icai_crl_gazebo](https://github.com/diegocubillo/icai_crl_gazebo): Contains custom plugins and the simulation configuration files for robot simulation.
- [icai_crl_description](https://github.com/diegocubillo/icai_crl_description): Contains the robot and environment description files.

### Install

Place this and the aforementioned repositories inside the src/ directory of a ROS 2 workspace and run command:
```bash
colcon build
```


## Launch Files

This package includes several launch files. The name convention is `environment_robot_controller_gui.launch.py`, where `environment` is the simulation environment, `robot` is the specific robot used, `controller` is the robot's actuator controller and `gui` is Gazebo's selected gui configuration. All launch files include a ROS 2 <-> Transport bridge of its topics.

Provided launch files are:

- `control_lab_kitt_dd_gui.launch.py`: Launches the robot simulation with ICAI's Control Laboratory environment, AMR `kitt` and Gazebo's confroller `diff_drive`.
- `control_lab_kitt_nav_dd_gui.launch.py`: Launches the robot simulation with ICAI's Control Laboratory environment, AMR `kitt_nav` and Gazebo's confroller `diff_drive`.
- `ramp_circuit_kitt_dd_gui.launch.py`: Launches the robot simulation with ICAI's competition circuit `ramp_circuit` environment, AMR `kitt` and Gazebo's confroller `diff_drive`.
- `launch_and_joy.launch.py`: Launches `control_lab_kitt_dd_gui.launch.py` with nodes from libraries `joy` and `teleop_twist_joy` configured to control `kitt` with FlySky FS-i6S radio controller.
- `demo_fpv_joy.launch.py`: Includes everything mentioned in `launch_and_joy.launch.py`, but using kitt_fpv, a variant that includes a front view camera. Its stream is visualized through `rqt_image_view`.

To launch them, source your workspace and run (substituting launch_file_name by your desired launch file):
```bash
ros2 launch icai_crl_bringup launch_file_name
```


## ROS Bridges

This package includes ROS 2 <-> Gazebo bridges for each type of robot. They are:

- `kitt_bridge.yaml`: Bridges velocity references, IMU data, distance sensors, odometry and tf messages.
- `kitt_fpv_bridge.yaml`: Adds a front camera stream to `kitt_bridge.yaml`.
- `kit_nav_bridge.yaml`: Adds a 360º LiDAR sensor to `kitt_bridge.yaml`.

## Configuration Files

This package includes the following configuration files:

- `gazebo_gui.config`: Defines the elements and configuration of Gazebo's GUI window.
- `gazebo_debug_gui.config`: GUI configuration with additional tools to analize the simulation.
- `FlySky_FS-i6S.config.yaml`: Contains the parameters to configure package `teleop_twist_joy` for FlySky FS-i6S radio controller. Left joystick commands linear velocity, right joystick commands angular velocity, left 3-way stick button enables movement in its lowest position, and right 2-way stick button boosts velocity references in its lowest position.