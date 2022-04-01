#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

from robot_model import *
from motor_driver import *



if __name__=="__main__":

    # Set device name
    device_name = '/dev/ttyUSB0'

    # Set initial angle
    dxl1_initial_pos = 2639
    dxl2_initial_pos = 2639
    dxl3_initial_pos = 2639

    # robot_model object
    robot_model = RobotModel()
    # motor_driver object
    motor_driver = MotorDriver(device_name)

    # Goal motor angle
    dxl1_goal_pos = []         
    dxl2_goal_pos = []
    dxl3_goal_pos = []

    # Target coordinate(XYZ, mm)
    target_coordinate = np.array([[100,   -50, -290],       # target 1
                                  [50,    100, -290],       # target 2
                                  [0,      50, -240],       # target 3
                                  [-50,  -100, -240],       # target 4
                                  [100,    50, -240], 
                                  [0,       0, -290]])     # target 5

    # solve IK and decide motor angle
    for i in range(len(target_coordinate)):
        thetas = robot_model.solve_ik(target_coordinate[i])

        angles = 2048 + thetas * 2048 / np.pi
        dxl1_goal_pos.append(int(angles[0]))
        dxl2_goal_pos.append(int(angles[1]))
        dxl3_goal_pos.append(int(angles[2]))

        print("Target[%d] Motor angles : [%d, %d, %d]" % (i+1, angles[0], angles[1], angles[2]))

        fig = robot_model.model_visualize(target_coordinate[i])
        plt.show()
        
        for j in range(3):
            if angles[j] > DXL_MINIMUM_POSITION_VALUE and angles[j] < DXL_MAXIMUM_POSITION_VALUE:
                continue
            else:
                print("Target[%d] : Motor angle [%d] is out of range!! (%03d)" % (i+1, j+1, angles[j]))
                quit()
    
    print("")


    # Motor setup
    motor_driver.connect()
    motor_driver.torque_enable()
    motor_driver.add_param_storage()

    # Read angle
    dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = motor_driver.sync_read_position()
    print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_pos, DXL2_ID, dxl2_present_pos, DXL3_ID, dxl3_present_pos))


    # Angle initialize
    initialize_speed = 30

    print("")
    print("Start initializing angle at [%d, %d, %d]." % (dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos))
    print("press any key to continue!")
    getch()
    print("")
    print("Angle initializing...")

    motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)


    # Write position control
    write_speed = 50

    for i in range(len(dxl1_goal_pos)):
        print("")
        print("[ID:%03d] GoalPos:%03d\t[ID:%03d] GoalPos:%03d\t[ID:%03d] GoalPos:%03d" % (DXL1_ID, dxl1_goal_pos[i], DXL2_ID, dxl2_goal_pos[2], DXL3_ID, dxl3_goal_pos[3]))
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break

        motor_driver.write_position(write_speed, dxl1_goal_pos[i], dxl2_goal_pos[i], dxl3_goal_pos[i])


    # Motor shutdown
    motor_driver.clear_param()
    motor_driver.torque_disable()
    motor_driver.disconnect()






    