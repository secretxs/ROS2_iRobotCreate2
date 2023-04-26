# ROS2_iCreateRobot
This repository contains a ROS2 Foxy workspace for a robot project that includes the `create_robot2` and  modified version of `joy_teleop` packages.

## Installation

#### Prerequisites
- [Ubuntu 20.4](https://www.releases.ubuntu.com/focal/)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- Ubuntu packages: `python3-rosdep`, `python3-colcon-common-extensions` 'g++'
  
  ```
  sudo apt install python3-rosdep python3-colcon-common-extensions, g++
  ```
- Camera driver and 2D Slam : `ros-foxy-usb-cam`, `ros-foxy-slam-toolbox`
  
  ```
  sudo apt-get install ros-foxy-usb-cam
  sudo apt install ros-foxy-slam-toolbox
  ```
- OpenCV (may take time)
  ```
  sudo curl -sSL https://raw.githubusercontent.com/milq/milq/master/scripts/bash/install-opencv.sh | sudo bash
  ```
  
#### Compiling
1. Clone this repository to your local machine:
   ``` bash
   cd ~
   git clone https://github.com/secretxs/ROS2_iRobotCreate2
   ```
2. Install dependencies
   ``` bash
   cd ROS2_iRobotCreate2/create_ws/src
   git clone https://github.com/AutonomyLab/libcreate
   cd ..
   rosdep update
   rosdep install --from-paths src -i
   ```
3. Build the workspace:
   ``` bash
   colcon build
   ```
4. Source the workspace: (add to shell startup)
   ``` bash
   echo "source ~/ROS2_iRobotCreate2/create_ws/install/setup.bash" >> ~/.bashrc
   ```
 

#### Setup

1. Connect computer to Create's 7-pin serial port
	- If using Create 1, ensure that nothing is connected to Create's DB-25 port

2. In order to connect to Create over USB, ensure your user is in the dialout group
``` bash
sudo usermod -a -G dialout $USER
```
3.  Logout and login for permission to take effect



## Running the robot
1. Launch the robot simulation:
   ``` bash
   ros2 launch create_bringup create_2.launch
   ```

Launch file arguments
   -   **config** - Absolute path to a configuration file (YAML). Default: `create_bringup/config/default.yaml`
   -   **desc** - Enable robot description (URDF/mesh). Default: `true`

For example, if you would like to disable the robot description and provide a custom configuration file:
``` bash
ros2 launch create_bringup create_2.launch config:=/abs/path/to/config.yaml desc:=false
```


2. Launch the joystick teleop node:
You need to modify joy_teleop file according to https://github.com/pgold/teleop_tools/commit/13488fcad84955a31deb608dd1829e90ac831a04
   
   ``` bash
   ros2 launch create_bringup joy_teleop.launch joy_config:=dualshock4
   ```

You can now control the robot using your joystick. 
