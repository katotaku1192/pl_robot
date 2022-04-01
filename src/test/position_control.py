#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def solve_ik(A,B,C,D,x,y,z):
    """ 角度を計算する関数 """
    # Read math function
    abs = np.abs
    cos = np.cos
    sin = np.sin
    pi = np.pi
    atan = np.arctan
    sqrt = np.sqrt

    # Decide angle
    thetas = np.zeros((3,))
    angles = np.array([pi * (2.0/3.0)*i for i in range(3)])
    for i in range(3):
        phi_0 = angles[i]
        # Solve inverse kinematics
        P = -A**2 + 2*A*D + 2*A*x*cos(phi_0) + 2*A*y*sin(phi_0) - B**2 + C**2 - D**2 - 2*D*x*cos(phi_0) - 2*D*y*sin(phi_0) - x**2 - y**2 - z**2
        Q = -2*B*z
        R = -2*A*B + 2*B*D + 2*B*x*cos(phi_0) + 2*B*y*sin(phi_0)
        theta_0 = -2*atan((Q - sqrt(-P**2 + Q**2 + R**2))/(P - R))
        theta_1 = -2*atan((Q + sqrt(-P**2 + Q**2 + R**2))/(P - R))
        # Choose smaller abs value
        thetas[i] = theta_1 if abs(theta_0) > abs(theta_1) else theta_0
    return thetas


def visualize(A,B,C,D,x,y,z):
    """ 可視化を行う関数 """
    # 可視化に必要なオブジェクト
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    
    # 単位ベクトル
    thetas = solve_ik(A,B,C,D,x,y,z)
    angles = np.array([np.pi * (2.0/3.0)*i for i in range(3)])
    unit_vectors = np.array([np.cos(angles),np.sin(angles),np.zeros(3)]).T
    ez = np.array([0,0,1])
    
    # 各座標の計算
    A_vectors = A * unit_vectors  
    B_vectors = np.array([A_vectors[i] + B*( unit_vectors[i] * np.cos(thetas[i])- ez * np.sin(thetas[i]) ) for i in range(3)])
    D_vector = np.array([x,y,z])      
    C_vectors = D_vector + D * unit_vectors
    
    # 描画
    index = np.arange(4) % 3
    ax.clear()
    ax.plot(xs=A_vectors[index,0],ys=A_vectors[index,1],zs=A_vectors[index,2],color='r')
    ax.plot(xs=C_vectors[index,0],ys=C_vectors[index,1],zs=C_vectors[index,2],color='g')
    
    for i in range(3):
        temp = [[A_vectors[i,j],B_vectors[i,j]]for j in range(3)]
        ax.plot(temp[0],temp[1],temp[2],color='b')
        
        temp = [[B_vectors[i,j],C_vectors[i,j]]for j in range(3)]
        ax.plot(temp[0],temp[1],temp[2],color='y')
    
    ax.set_xlim([-200,200])
    ax.set_ylim([-200,200])
    ax.set_zlim([-300,100])
    
    return fig



if __name__=="__main__":
    
    #********* DYNAMIXEL Model definition *********
    MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430

    # Control table address
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    BAUDRATE                    = 57600
    PROTOCOL_VERSION            = 2.0

    # Make sure that each DYNAMIXEL ID should have unique ID.
    DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
    DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
    DXL3_ID                     = 3                 # Dynamixel#1 ID : 3

    # Use the actual port assigned to the U2D2.
    DEVICENAME                  = '/dev/ttyUSB0'

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

    # Angle limit
    DXL_MINIMUM_POSITION_VALUE  = 2047              # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 3071              # Refer to the Maximum Position Limit of product eManual

    # Initial angles
    dxl_initial_position_1 = 2639
    dxl_initial_position_2 = 2639
    dxl_initial_position_3 = 2639

    #***************************************************


    #**************DICIDE TARGET POSITION***************

    # Set parameter of the robot
    A = 80.0
    B = 120.0
    C = 240.0
    D = 30.0

    # Goal motor angle
    dxl_goal_position_1 = []         
    dxl_goal_position_2 = []
    dxl_goal_position_3 = []

    # Target coordinate(XYZ, mm)
    target_coordinate = np.array([[100,   -50, -290],       # target 1
                                  [50,    100, -290],       # target 2
                                  [0,    50, -240],       # target 3
                                  [-50, -100, -240],       # target 4
                                  [100,   50, -240], 
                                  [0,   0, -290]])     # target 5

    # solve IK and decide motor angle
    for t in range(len(target_coordinate)):
        thetas = solve_ik(A, B, C, D, target_coordinate[t][0], target_coordinate[t][1], target_coordinate[t][2])
        angles = 2048 + thetas * 2048 / np.pi
        dxl_goal_position_1.append(int(angles[0]))
        dxl_goal_position_2.append(int(angles[1]))
        dxl_goal_position_3.append(int(angles[2]))

        print("Target[%d] Motor angles : [%d, %d, %d]" % (t+1, angles[0], angles[1], angles[2]))

        fig = visualize(A, B, C, D, target_coordinate[t][0], target_coordinate[t][1], target_coordinate[t][2])
        plt.show()
        
        for i in range(3):
            if angles[i] > DXL_MINIMUM_POSITION_VALUE and angles[i] < DXL_MAXIMUM_POSITION_VALUE:
                continue
            else:
                print("Target[%d] : Motor angle [%d] is out of range!! (%03d)" % (t+1, i+1, angles[i]))
                quit()
    
    print("")

    #*****************************************************


    #****************MOTOR SETUP *************************

    # Initialize PortHandler instance
    portHandler = PortHandler(DEVICENAME)
    # Initialize PacketHandler instance
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()


    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL1_ID)

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL2_ID)

    # Enable Dynamixel#3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL3_ID)


    # Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)
        quit()

    # Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
        quit()

    # Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL3_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL3_ID)
        quit()




    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    #print("hoge")
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
        quit()

    # Check if groupsyncread data of Dynamixel#2 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
        quit()

    # Check if groupsyncread data of Dynamixel#2 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncRead getdata failed" % DXL3_ID)
        quit()
    

    # Get Dynamixel#1 present position value
    dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    # Get Dynamixel#2 present position value
    dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    # Get Dynamixel#2 present position value
    dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position, DXL3_ID, dxl3_present_position))


    #***************************************************



    # Angle initialize************************************************

    print("")
    print("Start initializing angle at [%d, %d, %d]." % (dxl_initial_position_1, dxl_initial_position_2, dxl_initial_position_3))
    print("press any key to continue!")
    getch()

    print("")
    print("Angle initializing...")

    # Rotate motor slowly
    for i in range(0, 4096):
        if i%30==0 or i==4095: 
            g1 = dxl1_present_position + (dxl_initial_position_1 - dxl1_present_position) * i / 4095
            g2 = dxl2_present_position + (dxl_initial_position_2 - dxl2_present_position) * i / 4095
            g3 = dxl3_present_position + (dxl_initial_position_3 - dxl3_present_position) * i / 4095

            param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(g1)), DXL_HIBYTE(DXL_LOWORD(g1)), DXL_LOBYTE(DXL_HIWORD(g1)), DXL_HIBYTE(DXL_HIWORD(g1))]
            param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(g2)), DXL_HIBYTE(DXL_LOWORD(g2)), DXL_LOBYTE(DXL_HIWORD(g2)), DXL_HIBYTE(DXL_HIWORD(g2))]
            param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(g3)), DXL_HIBYTE(DXL_LOWORD(g3)), DXL_LOBYTE(DXL_HIWORD(g3)), DXL_HIBYTE(DXL_HIWORD(g3))]

            # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position_1)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                quit()

            # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                quit()

            # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position_3)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
                quit()

            # Syncwrite goal position
            dxl_comm_result = groupSyncWrite.txPacket()

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Clear syncwrite parameter storage
            groupSyncWrite.clearParam()

            time.sleep(0.001)

    #**************************************************************************



    # Write and read
   

    for pos_num in range(len(dxl_goal_position_1)):    

        print("")
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break

        # Rotate motor slowly
        for i in range(0, 4096):
            if i%40==0 or i==4095: 
                # Allocate goal position value into byte array 
                if pos_num == 0:
                    g1 = dxl_initial_position_1 + (dxl_goal_position_1[0] - dxl_initial_position_1) * i / 4095
                    g2 = dxl_initial_position_2 + (dxl_goal_position_2[0] - dxl_initial_position_2) * i / 4095
                    g3 = dxl_initial_position_3 + (dxl_goal_position_3[0] - dxl_initial_position_3) * i / 4095
                else:
                    g1 = dxl_goal_position_1[pos_num-1] + (dxl_goal_position_1[pos_num] - dxl_goal_position_1[pos_num-1]) * i / 4095
                    g2 = dxl_goal_position_2[pos_num-1] + (dxl_goal_position_2[pos_num] - dxl_goal_position_2[pos_num-1]) * i / 4095
                    g3 = dxl_goal_position_3[pos_num-1] + (dxl_goal_position_3[pos_num] - dxl_goal_position_3[pos_num-1]) * i / 4095        
    
                param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(g1)), DXL_HIBYTE(DXL_LOWORD(g1)), DXL_LOBYTE(DXL_HIWORD(g1)), DXL_HIBYTE(DXL_HIWORD(g1))]
                param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(g2)), DXL_HIBYTE(DXL_LOWORD(g2)), DXL_LOBYTE(DXL_HIWORD(g2)), DXL_HIBYTE(DXL_HIWORD(g2))]
                param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(g3)), DXL_HIBYTE(DXL_LOWORD(g3)), DXL_LOBYTE(DXL_HIWORD(g3)), DXL_HIBYTE(DXL_HIWORD(g3))]

                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position_1)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                    quit()

                # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                    quit()

                # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position_3)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
                    quit()

                # Syncwrite goal position
                dxl_comm_result = groupSyncWrite.txPacket()

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

                # Clear syncwrite parameter storage
                groupSyncWrite.clearParam()


        time.sleep(0.1)

        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#3 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL3_ID)
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # Get Dynamixel#3 present position value
        dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position_1[pos_num], dxl1_present_position, DXL2_ID, dxl_goal_position_2[pos_num], dxl2_present_position, DXL3_ID, dxl_goal_position_3[pos_num], dxl3_present_position))

    print("")
    print("Press any key to exit!")
    getch()

    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


    # Close port
    portHandler.closePort()




