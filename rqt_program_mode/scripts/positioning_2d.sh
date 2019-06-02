#!/bin/bash

gnome-terminal --tab -e "bash -c 'printf \"\033]0;Kuka Controller\007\"; roslaunch kuka_hw_cart_vel test.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Coil Controller\007\"; roslaunch coil_controller coilPidControl.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Camera Bottom\007\"; roslaunch image_processing camera_node.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Image Visualization\007\"; roslaunch image_processing visualization.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Positioning2d Node\007\"; rosrun image_processing object_positioning'" \
               --tab -e "bash -c 'printf \"\033]0;Goal Visualize Node\007\"; rosrun image_processing goalVisualizeArray.py'" \