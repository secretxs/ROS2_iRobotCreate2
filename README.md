# ROS2_iCreateRobot

This ROS2_iRobotCreate2 repository provides an integration of Object Detection and Monocular Depth Estimation in ROS2 Foxy ecosystem and support to visualize on RViz2. It has been optimized for the Jetson Xavier NX dev kit, the iCreate Robot 2, and the Logitech c920 Monocular Camera.

## Features

- **Complete Integration with ROS2 Ecosystem**: This repository provides seamless integration with the ROS ecosystem. It supports video feed streaming, the usage of jetson-inference library nodes, robot control, and teleoperation.
- **Custom Docker Container**: Included in the repository is a custom Docker container designed for running and visualizing Object Detection and Monocular Depth Estimation in ROS2 Foxy using [jetson-inferece](https://github.com/dusty-nv/jetson-inference) in modified container of [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning)that has been specifically tailored to work with NVIDIA Jetson devices and TensorRT.
- **RVIZ2 Configuration File**: A configuration file for RVIZ2 is provided, allowing users to visualize the entire process.
  
Please feel free to contribute, make suggestions, or raise issues if you encounter any problems. We hope you find this project useful, and we look forward to seeing the innovative ways you use it in your own applications.

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
   cd ..
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
##### Camera
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
   - **desc** - Enable robot description (URDF/mesh). Default: `true`
     
2. Launch the joystick teleop node (to robot using your joystick) \*optional
   ``` bash
   ros2 launch create_bringup joy_teleop.launch joy_config:=dualshock4
   ```
Warning! You may need to modify joy_teleop file according to https://github.com/pgold/teleop_tools/commit/13488fcad84955a31deb608dd1829e90ac831a04


If you want to learn more details about IRobot Create 2, you can go to my own fork of [create_robot](https://github.com/secretxs/create_robot) repository or original repository [create_robot](https://github.com/AutonomyLab/create_robot).

## Running the Custom Docker for Object Detection and Monocular Depth Estimation and Visualization
You can follow the next steps to use modified version of [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) docker container that works in ROS ecosystem that uses
- Object Detection using Detectnet with "ssd-mobilenet-v2"  
- Monocular Depth Estimation and Visualization with "fcn-mobilenet"
If you you more customization, you can go [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) for more customizable and purpose specific dockers that natively supports some of the image processing ROS nodes or write your own ROS node using [jetson-inference](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md).
1. Run the ROS USB Camera Node:
   ``` bash
   ros2 run usb_cam usb_cam_node_exe
   ```
2. Run the bash script to pull and run docker container.
   ``` bash
   docker/launch/run.sh
   ```
3. Run the pre-configured Object Detection Script
    ``` bash
   python3 ROS2_iRobotCreate2/docker/scripts/ros_object.py 
   ```
4. Run the pre-configured Monocular Depth Estimation Script
    ``` bash
   python ROS2_iRobotCreate2/docker/scripts/ros_depth.py
   ```

## Visualize on Rviz2
You can launch pre-configured rviz2 with
```
ros2 run rviz2 rviz2 -d ~/ROS2_iRobotCreate2/config/rviz2_icreate2.rviz
```
