#!/bin/bash
# 启动gmapping建图
gnome-terminal \
--tab -e "zsh -c 'source $HOME/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch hiwonder_slam slam.launch robot_name:=/ master_name:=/'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 30;roscd hiwonder_slam/rviz;rviz rviz -d gmapping_desktop.rviz'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 30;roslaunch hiwonder_peripherals teleop_key_control.launch robot_name:=/'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 30;rosrun hiwonder_slam map_save.py'"
