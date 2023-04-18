# ROS2_iCreateRobot
This repository contains a ROS2 Foxy workspace for a robot project that includes the `create_robot2` and  modified version of `joy_teleop` packages.

## Installation

#### Prerequisites
- [Ubuntu 20.4](https://www.releases.ubuntu.com/focal/)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- Ubuntu packages: `python3-rosdep`, `python3-colcon-common-extensions`
  
  ```
  sudo apt install python3-rosdep python3-colcon-common-extensions
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
   cd ROS2_iRobotCreate2
   rosdep update
   rosdep install --from-paths src -i
   ```
3. Build the workspace:
   ``` bash
   colcon build
   ```
4. Source the workspace:
   ``` bash
   source ~/ROS2_iCreateRobot/install/setup.bash
   ```
 

#### Setup
1. After compiling from source, don't forget to source your workspace:  
    ``` bash
    source ~/create_ws/install/setup.bash
    ```

2. Connect computer to Create's 7-pin serial port
	- If using Create 1, ensure that nothing is connected to Create's DB-25 port

4. In order to connect to Create over USB, ensure your user is in the dialout group
``` bash
sudo usermod -a -G dialout $USER
```
4.  Logout and login for permission to take effect



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
   
   ``` bash
   ros2 launch joy_teleop joy_teleop.launch.py joy_config:=dualshock4
   ```

You can now control the robot using your joystick. 
