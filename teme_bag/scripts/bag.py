#!/usr/bin/python
import numpy as np
from subprocess import PIPE, Popen
import argparse

parser = argparse.ArgumentParser(prog='Teme Logging')
parser.add_argument('-a', '--all', help='Log all files', action='store_true')

if __name__ == '__main__':
    source = "/home/aytac/Desktop/meta_data.txt"

    exp_name = raw_input("Experiment name:")
    liquid_number = raw_input("Liquid number:")
    distance = raw_input("Distance between coils:")
    microrobot_number = raw_input("Microrobot number:")

    meta_data = 'Experiment name:{}\nLiquid number:{}\nDistance between coils:{}\nMicrorobot number:{}'.format(exp_name,
                                                                                                               liquid_number,
                                                                                                               distance,
                                                                                                               microrobot_number)                                                                                                          
    file = open(source, 'w')

    args = parser.parse_args()

    topics = ["/joint_states", "/dynamixel/joint_states", "/coil_states", "/camera_node_bottom/position_dist"]

    if(args.all == 1):
        file.write(meta_data)
    else:
        print "all not"
    
    
    p = Popen(['ls', '-a'])
    p.communicate()
    file.close()