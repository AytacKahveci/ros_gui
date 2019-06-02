#!/bin/bash

gnome-terminal --tab -e "bash -c 'printf \"\033]0;Kuka Controller\007\"; roslaunch kuka_hw_cart_vel test.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Coil Controller\007\"; roslaunch coil_controller coilPidControl.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Camera Front\007\"; roslaunch image_processing camera_node_front.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Visulaziton\007\"; roslaunch image_processing visualization.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Dynamixel Controller\007\"; roslaunch dynamixel_custom_controller commonControl.launch'" \
               --tab -e "bash -c 'printf \"\033]0;Positioning3d Node\007\"; rosrun image_processing object_positioning3d'" \