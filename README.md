# ROS2_iCreateRobot
This repository contains a ROS2 Foxy workspace for a robot project that includes the `create_robot2` and  modified version of `joy_teleop` packages.

## Installation

#### Prerequisites
- [Ubuntu 20.4](https://www.releases.ubuntu.com/focal/)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Docker](https://docs.docker.com/get-docker/)
- Ubuntu packages: `python3-rosdep`, `python3-colcon-common-extensions`, `g++`
  
  ```
  sudo apt install python3-rosdep python3-colcon-common-extensions g++
  ```
  
### Compiling ROS2 Workspace
1. Clone this repository to your local machine:
   ``` bash
   cd ~
   git clone --recursive https://github.com/secretxs/ROS2_iRobotCreate2
   ```
2. Install dependencies
   ``` bash
   cd ROS2_iRobotCreate2/create_ws/src
   rosdep update
   rosdep install --from-paths src -i
   ```
3. Build the workspace:
   ``` bash
   colcon build
   ```
3. Source the workspace:
   ``` bash
   source ~/ROS2_iRobotCreate2/create_ws/install/setup.bash
   ```
4. *Optional if you don't want to source everytime:
   ``` bash
   echo "source ~/ROS2_iRobotCreate2/create_ws/install/setup.bash" >> ~/.bashrc
   ```

## Hardware Setup

##### Create 2
1. Connect computer to Create's 7-pin serial port
	- If using Create 1, ensure that nothing is connected to Create's DB-25 port

2. In order to connect to Create over USB, ensure your user is in the dialout group
``` bash
sudo usermod -a -G dialout $USER
```
3.  Logout and login for permission to take effect

#### Camera
This project supports streaming video feeds and images via a variety of interfaces and protocols that is supported by [jetson-inference](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md)
-   [MIPI CSI cameras](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#mipi-csi-cameras)
-   [V4L2 cameras](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#v4l2-cameras)
-   [WebRTC](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#webrtc)
-   [RTP](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#rtp) / [RTSP](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#rtsp)
-   [Videos](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#video-files) & [Images](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#image-files)
-   [Image sequences](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#image-files)
-   [OpenGL windows](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md#output-streams)

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

## Running the Custom ros_deep_learning Docker for Object Detection
[ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) contains DNN inference nodes and camera/video streaming nodes for ROS/ROS2 with support for NVIDIA **[Jetson Nano / TX1 / TX2 / Xavier / Orin](https://developer.nvidia.com/embedded-computing)** devices and TensorRT.
You can go [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) for more customizable and purpose specific docker or follow next steps for my pre-configured for object detection ][custom docker](https://hub.docker.com/repository/docker/secretxs/foxy-pytorch-l4t-r35.2.1/general)
*Be careful about running docker from the script below. It passes parameters (camera etc.) that are required to pass to docker to able function properly. 

1. Run the bash script to pull and run docker container.
   ``` bash
   docker/run.sh
   ```
2. Run the object detection package.
    ``` bash
   ros2 launch ros_deep_learning detectnet.ros2.launch input:=/dev/video0 output:=display://0 model_name:="ssd-mobilenet-v2"
   ```

## Visualize on Rviz2
You can launch pre-configured rviz2 by;
```
ros2 run rviz2 rviz2 -d ~/ROS2_iRobotCreate2/config/rviz2_icreate2.rviz
```
