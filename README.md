
# LAB 9

## INTRODUCTION

The Purpose of this lab experiment is to use the calibrated camera to get the world coordinates of a cube.

## Steps to Perform Pick and Place

1. Run the following command to download files for this lab into your folder:

```console
git clone https://github.com/ENRE467/Lab_9.git
```

2. Navigate to Lab_9/src directory using the cd command.

3. Run the following command so that you can see the GUI applications from docker container in the screen of the host pc:  

```console
xhost +local:docker
```

4. Run the following command to run the docker container: 

```console
docker run -it --rm --name UR3Container --net=host --pid=host --privileged --env="DISPLAY=$DISPLAY" --volume="$PWD:/home/${USER}/workspace/src" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/dev:/dev:rw" --ulimit rtprio=99 --ulimit rttime=-1 ur3e_image:latest
```

5. Run the following commands:

```console
Catkin build
```

```console
source ~/workspace/devel/setup.bash
```

6. Using tmux, split the terminal window into multiple terminals.

7. Start up the real ur3e robot using the tablet and run the following commands in order in different terminals:

```
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.22 kinematics_config:=$(rospack find ur_calibration)/calib/ur3e_calib.yaml z_height:=0.766 gripper:=true
```
start Moveit! for UR3e:

```console
roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch
```

Start rviz:

```console
roslaunch ur3e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur3e_moveit_config)/launch/moveit.rviz
```

8. Now run the following command in a different terminal to start the camera and the ArUco tag tracking functionality:

```console
roslaunch camera_calib_pkg extrinsic_calibration.launch aruco_tracker:=true   show_output:=true
```

9. run the following command to load your saved calibration

```
roslaunch camera_calib_pkg aruco_tf.launch load_calibration:=true 
```

10. Initiate UR Driver Communication with real robot with gripper
```console

roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.22 kinematics_config:=$(rospack find ur_calibration)/calib/ur3e_calib.yaml z_height:=0.766 use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR
```
11. Run the 2-Finger Gripper Driver Node

```console
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
```

12. Run the 2-Finger Gripper Custom controller Node

```console
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```

13. To run the pick and place node : 
```console
rosrun pick_and_place pick_place
```



