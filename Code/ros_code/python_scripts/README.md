
# ROS Python Scripts and Launch Files

This repository contains a set of ROS scripts and launch files designed for module control, visualization, and VICON data processing.

## Notes

### Dependencies

- **ROS Noetic**: The main middleware used for the robotic applications.
- **Ubuntu 20.04 LTS**: The supported operating system for the ROS Noetic version.
- **vicon_bridge**: A ROS package allowing to use the VICON motion capture system.

To install the required dependencies, follow the steps below:

```console
cd ~/catkin_ws/src/
git clone https://github.com/ethz-asl/vicon_bridge
rosdep install -r --from-paths vicon_bridge/
cd ~/catkin_ws/
catkin_make
```

Remember to set up the VICON system by modifying the ports in the `vicon.launch` file as per your setup. In addition, all the vicon objects for the modules must be named `module_n` where n is the module ID. This will result in the ROS topic `/vicon/module_n/module_n` which the Python scripts can subscribe to. The payload in the scripts are currently named as `payload_box`.
The hook and claw of module (ID=n) is named `module_n_connector` and `module_n_claw`, respectively. Naming of vicon objects can be done from Vicon Software.

### Scripts Overview

1. **pid_position.py**: Implements a PID position controller for robotic modules. It uses the difference between a module's current pose and its goal pose to compute control commands.
2. **pid_position_connect.py**: Acts as a connector controller, adjusting the orientation of a module's connector in relation to another module's connector. It's essential for ensuring modules connect correctly.
3. **set_goal_to_payload_box.py**: Dynamically sets a movement goal for a module based on the real-time position of a designated payload box. This ensures the module stays aligned with moving targets.
4. **test_move.py**: A testing utility. It helps in moving and connecting different modules to verify their operational status and connectivity capabilities.
5. **vicon_to_odom.py**: A utility that transforms VICON system's data into odometry data, making it suitable for navigation and control tasks in ROS.
6. **vicon_to_pointcloud.py**: Converts VICON data into a point cloud representation, useful for visualization and environment mapping.
7. **wifi_communication.py**: Manages WiFi communication with ESP32 hardware. This script is responsible for sending control commands and receiving data via WiFi.

### Launch Files Overview

1. **launch_basic.launch**: A foundational launch file that initializes the necessary ROS nodes. It sets up the basic environment for other functionalities to work.
2. **map_server_start.launch**: Gets the map server running, ensuring the robotic modules have a map to navigate within.
3. **navigation.launch**: Facilitates the navigation functionalities for the modules, making them move and achieve their goals.
4. **start_all_navigation.launch**: A comprehensive launch file that activates all navigation-related functionalities, ensuring modules can navigate, avoid obstacles, and reach their destinations.
5. **vicon_data_processing.launch**: Handles the data received from the VICON motion capture system, transforming it into usable ROS messages.
6. **visualization.launch**: Initiates RVIZ with configurations suitable for visualizing the robotic modules, their trajectories, and the environment.

### Usage

For basic setup and module visualization with RVIZ:

```console
roslaunch python_scripts launch_basic.launch 
```

For complete navigation functionalities:

```console
roslaunch python_scripts start_all_navigation.launch
```

For processing and using VICON motion capture data:

```console
roslaunch python_scripts vicon_data_processing.launch
```

---
