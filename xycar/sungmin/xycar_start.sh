#! /bin/bash
export ROS_MASTER_URI=http://192.168.0.118:11311
export ROS_HOSTNAME=192.168.0.118
source /home/nvidia/xycar/ros/devel/setup.sh

sleep 8
ls -l /dev/tty*
ls -l /dev/video*
roslaunch racecar teleop.launch

