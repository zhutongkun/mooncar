#!/bin/bash
source $HOME/ros_ws/src/hiwonder_bringup/scripts/source_env.bash

roslaunch /home/hiwonder/ros_ws/src/hiwonder_driver/hiwonder_controller/launch/hiwonder_controller.launch &
sleep 10
roslaunch /home/hiwonder/ros_ws/src/hiwonder_peripherals/launch/depth_cam.launch &
sleep 10
roslaunch /home/hiwonder/ros_ws/src/hiwonder_app/launch/start_app.launch &
sleep 10
roslaunch /home/hiwonder/ros_ws/src/hiwonder_bringup/launch/bringup.launch
