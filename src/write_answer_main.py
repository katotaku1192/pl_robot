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
    dxl1_initial_pos = 2400
    dxl2_initial_pos = 2400
    dxl3_initial_pos = 2400

    # robot_model object
    robot_model = RobotModel()
    # motor_driver object
    motor_driver = MotorDriver(device_name)
    # trajectory_func object
    trajectory_func = TrajectoryFunc()

    # Pos control angles
    dxl1_goal_pos = []         
    dxl2_goal_pos = []
    dxl3_goal_pos = []

    # Motor setup
    motor_driver.connect()
    motor_driver.torque_enable()
    motor_driver.add_param_storage()


    # Read angle
    dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = motor_driver.sync_read_position()
    print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_pos, DXL2_ID, dxl2_present_pos, DXL3_ID, dxl3_present_pos))


    # 原点へ移動
    initialize_speed = 50

    print("press any key to continue!")
    getch()
    print("")
    print("Angle initializing...")

    motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)



    # 問題を解く間、エンドエフェクタを避けておく
    # 避けるときのエンドエフェクタの座標
    solving_pos = [150, 0, -170]
    # 逆運動学
    theta = robot_model.solve_ik(solving_pos)
    angle =  2048 + theta * 2048 / np.pi
    dxl1_solving_pos = int(angle[0])
    dxl2_solving_pos = int(angle[1])
    dxl3_solving_pos = int(angle[2])
    # モータを動かす
    print("press any key to continue!")
    getch()
    print("")
    avoid_speed = 50
    motor_driver.write_position(avoid_speed, dxl1_solving_pos, dxl2_solving_pos, dxl3_solving_pos)



    ######################
    # ここで間違い探しを解く #
    ######################

    # Answer list（後々は画像から求める）
    answer_centers = [(-72.75, 13.75), (-48.5, -48.75), (-26.75, 52.25), (-18.0, -53.0), (-13.25, 15.5), (25.0, -4.25), (51.25, 60.0), (59.0, -67.5)]
    answer_radius = [7.25, 10.0, 20.25, 14.5, 19.5, 12.25, 10.5, 8.75]
    # R=10未満の場合は10にする
    for i in range(len(answer_radius)):
        if answer_radius[i] < 10.0:
            answer_radius[i] = 10.0


    # 解答から丸をつける位置の座標リスト（初期値）を計算(XYZ, mm)
    target_coordinate = np.empty((len(answer_centers)*2, 3))

    for i in range(len(answer_centers)):
        # Move pos
        target_coordinate[2*i][0] = answer_centers[i][0] + answer_radius[i]
        target_coordinate[2*i][1] = answer_centers[i][1]
        target_coordinate[2*i][2] = -230.0
        
        target_coordinate[2*i+1][0] = answer_centers[i][0] + answer_radius[i]
        target_coordinate[2*i+1][1] = answer_centers[i][1]
        target_coordinate[2*i+1][2] = -230.0

    print(target_coordinate)



    # 円軌道を描くモーター角を保存
    circles_data = []
    for i in range(len(answer_centers)):
        center = [answer_centers[i][0], answer_centers[i][1], -240.0]  # 書くときは-270.0
        circle_angles = trajectory_func.xy_circle(center, answer_radius[i])
        circle_angles = np.tile(circle_angles, (2,1))
        circles_data.append(circle_angles)
    
    

    # 丸をつける位置の座標リストを逆運動学でモータ角にする
    for i in range(len(target_coordinate)):
        thetas = robot_model.solve_ik(target_coordinate[i])

        angles = 2048 + thetas * 2048 / np.pi
        dxl1_goal_pos.append(int(angles[0]))
        dxl2_goal_pos.append(int(angles[1]))
        dxl3_goal_pos.append(int(angles[2]))

        print("Target[%d] Motor angles : [%d, %d, %d]" % (i+1, angles[0], angles[1], angles[2]))

        #fig = robot_model.model_visualize(target_coordinate[i])
        #plt.show()
        
        for j in range(3):
            if angles[j] > DXL_MINIMUM_POSITION_VALUE and angles[j] < DXL_MAXIMUM_POSITION_VALUE:
                continue
            else:
                print("Target[%d] : Motor angle [%d] is out of range!! (%03d)" % (i+1, j+1, angles[j]))
                quit()
    
    print("")


    
    # 原点へ移動
    initialize_speed = 50

    print("press any key to continue!")
    getch()
    print("")
    print("Angle initializing...")

    motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)



    # Write position control
    move_speed = 100
    write_speed = 60

    for i in range(len(dxl1_goal_pos)):
        print("")
        print("[ID:%03d] GoalPos:%03d\t[ID:%03d] GoalPos:%03d\t[ID:%03d] GoalPos:%03d" % (DXL1_ID, dxl1_goal_pos[i], DXL2_ID, dxl2_goal_pos[2], DXL3_ID, dxl3_goal_pos[3]))
        print("Press any key to continue! (or press ESC to quit!)")
        #if getch() == chr(0x1b):
        #    break

        # Move to next position
        motor_driver.write_position(move_speed, dxl1_goal_pos[i], dxl2_goal_pos[i], dxl3_goal_pos[i])
        
        # Write circles
        if i % 2 == 0:
            print("Press any key to continue! (or press ESC to quit!)")
        #    if getch() == chr(0x1b):
        #        break
            motor_driver.write_trajectory(write_speed, circles_data[i/2])



    # 原点へ移動
    motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)
    print("Press any key to exit!")
    getch()


    # Motor shutdown
    motor_driver.clear_param()
    motor_driver.torque_disable()
    motor_driver.disconnect()