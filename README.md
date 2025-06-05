
# AMR BT Simulation

This repository is a ros2 workspace consisting of packages to simulate and control an AMR using behavior trees. Tested using **ROS2 Humble** and **Gazebo 11** running on Ubuntu 22.04.
## [Demonstration Video](https://drive.google.com/file/d/1-gM_xgbaBwFRFHjFdm1s99qfZk8QfYt5/view?usp=sharing)
https://drive.google.com/file/d/1-gM_xgbaBwFRFHjFdm1s99qfZk8QfYt5/view?usp=sharing

## Dependencies:
ROS2 Humble
Gazebo Classic
Nav2
behaviortree_cpp_v3

## Setup
Clone the repository "akarsh_bt_ws" repository:

    git clone https://github.com/akarsh2906/akarsh_bt_ws.git


## Build the workspace
In the root directory of the workspace

    colcon build
Source the workspace.

    source ~/akarsh_bt_ws/install/setup.bash
## Launch
### Launch the simulation and Nav2 using these two commands
Launch the sim:

    ros2 launch diff_bot diff_bot_gazebo.launch.py
Start Nav2:

    ros2 launch diff_bot diff_bot_navigation.launch.py


### Launch the autonomy script which uses behavior tree


    ros2 launch amr_bt autonomy_launch.py

### Locations
The locations are set in the "station_locations.yaml" file present in the "config" directory of "amr_bt" package.

### "amr_bt" package contains:
**autonomy_node.cpp** which is responsible for creating and ticking the behavior tree.
**navigation_behaviors.cpp** which contains all the functionality for the BT nodes implemented.
**battery_sim_node.cpp** which simulates the battery percentage reducing and also charging using a ros2 service.
