Racecar Simulator
=================
MIT-Racecar Gazebo simulation ported to ROS 2 Foxy Fitzroy.

## Getting Started

This code was developed and tested in a machine running [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) and [Gazebo 9](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) on Ubuntu 20.04.

### Dependencies
- [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
- [xacro](https://pypi.org/project/xacro/)
- [ros2_controllers](https://index.ros.org/r/ros2_controllers/github-ros-controls-ros2_controllers/)

### Installation
1. Clone the repository to the src directory of your ROS2 workspace:
  ```
  git clone https://github.com/devanshdhrafani/racecar_gazebo.git
  ```
2. Move back to the root of your workspace and build the packages:
  ```
  cd ..
  colcon build --symlink-install
  ```
### Launching the simulator
The racecar can be launched in an empty gazebo world using:
```
ros2 launch racecar_gazebo racecar.launch.py
```

## Code structure
This package retains the file structure of [mit-racecar/racecar_gazebo](https://github.com/mit-racecar/racecar_gazebo) while making a few changes to follow ROS2 standards.

- ```racecar_control```: This package containes the config and launch files for the controllers.
- ```racecar_description```: This package stores the URDF, meshes, STLs and other model files describing the racecar.
- ```racecar_gazebo```: This package containes launch files for starting the gazebo simulation.

