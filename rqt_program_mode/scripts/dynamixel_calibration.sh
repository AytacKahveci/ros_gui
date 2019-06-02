#!/bin/bash

gnome-terminal --tab -e "bash -c 'printf \"\033]0;DynamixelController\007\"; roslaunch dynamixel_custom_controller dynamixel_limits.launch'" \
               --tab -e "bash -c 'printf \"\033]0;DynamixelLimitsNode\007\"; rosrun dynamixel_custom_controller dynamixel_limits_node'"