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
from trajectory_func import *



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
    # trajectory_func object
    trajectory_func = TrajectoryFunc()


    # Get tragectory(circle) positions #1
    center = [0, 0, -200]
    radius = 100
    circle_1 = trajectory_func.xy_circle(center, radius)
    circle_1 = np.tile(circle_1, (3,1))
    print("Goal positions #1")
    print(np.shape(circle_1))

    # # Get tragectory(circle) positions #1
    circle_2 = trajectory_func.xy_circle(center, radius, direction="r")
    circle_2 = np.tile(circle_2, (3,1))
    print("Goal positions #2")
    print(np.shape(circle_2))


    # Motor setup
    motor_driver.connect()
    motor_driver.torque_enable()
    motor_driver.add_param_storage()

    # Read angle
    dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = motor_driver.sync_read_position()
    print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_pos, DXL2_ID, dxl2_present_pos, DXL3_ID, dxl3_present_pos))


    # Angle initialize
    initialize_speed = 100

    print("")
    print("Start initializing angle at [%d, %d, %d]." % (dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos))
    print("press any key to continue!")
    getch()
    print("")
    print("Angle initializing...")

    motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)


    # Write tragectory
    write_speed = 20

    print("")
    print("Write circle")
    print("press any key to continue!")
    getch()
    motor_driver.write_trajectory(write_speed, circle_1)
    motor_driver.write_trajectory(write_speed, circle_2)

    # Motor shutdown
    motor_driver.clear_param()
    motor_driver.torque_disable()
    motor_driver.disconnect()










    