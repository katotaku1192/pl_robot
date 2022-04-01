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
    dxl1_initial_pos = 2400
    dxl2_initial_pos = 2400
    dxl3_initial_pos = 2400

    # robot_model object
    robot_model = RobotModel()
    # motor_driver object
    motor_driver = MotorDriver(device_name)

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

    getch()

    # Motor shutdown
    motor_driver.clear_param()
    motor_driver.torque_disable()
    motor_driver.disconnect()