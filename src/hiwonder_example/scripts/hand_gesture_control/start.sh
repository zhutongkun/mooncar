#!/bin/bash
gnome-terminal \
--tab -e "zsh -c 'source $HOME/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch hiwonder_example hand_gesture_control.launch'" 
