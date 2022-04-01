#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import numpy as np
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



#********* DYNAMIXEL Model definition **********************************

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

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Angle limit
DXL_MINIMUM_POSITION_VALUE  = 1800              # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 3071              # Refer to the Maximum Position Limit of product eManual

#**********************************************************************



class MotorDriver:

    def __init__(self, device_name):

        # Use the actual port assigned to the U2D2.
        self.device_name = device_name

        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.device_name)

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        
        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)


    def connect(self):
        """ U2D2に接続 """
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()


    def disconnect(self):
        """ 接続を解除 """
        self.portHandler.closePort()


    def torque_enable(self):
        """ 3つのモータのトルクをON """
        for i in range(3):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i+1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % (i+1))


    def torque_disable(self):
        """ 3つのモータのトルクをOFF """
        for i in range(3):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i+1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    
    def add_param_storage(self):
        """ 3つのモータの現在位置のパラメータを格納するストレージを確保 """
        for i in range(3):
            dxl_addparam_result = self.groupSyncRead.addParam(i+1)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % i+1)
                quit()

    
    def clear_param(self):
        """ パラメータを格納するストレージをクリア """
        self.groupSyncRead.clearParam()


    def sync_read_position(self):
        """ 3つのモータの現在位置を取得 """
        # Syncread present position
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        for i in range(3):
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupSyncRead.isAvailable(i+1, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % i+1)
                quit()

        # Get Dynamixel#1 present position value
        dxl1_present_pos = self.groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        # Get Dynamixel#2 present position value
        dxl2_present_pos = self.groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        # Get Dynamixel#2 present position value
        dxl3_present_pos = self.groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        return dxl1_present_pos, dxl2_present_pos, dxl3_present_pos


    def angle_initialize(self, speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos):
        """ モータの角度を初期化 """
        # Read present position
        dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = self.sync_read_position()

        # Rotate motor slowly
        for i in range(0, 4096):
            if i % speed==0 or i==4095: 
                g1 = int(dxl1_present_pos + (dxl1_initial_pos - dxl1_present_pos) * i / 4095)
                g2 = int(dxl2_present_pos + (dxl2_initial_pos - dxl2_present_pos) * i / 4095)
                g3 = int(dxl3_present_pos + (dxl3_initial_pos - dxl3_present_pos) * i / 4095)

                param_goal_pos_1 = [DXL_LOBYTE(DXL_LOWORD(g1)), DXL_HIBYTE(DXL_LOWORD(g1)), DXL_LOBYTE(DXL_HIWORD(g1)), DXL_HIBYTE(DXL_HIWORD(g1))]
                param_goal_pos_2 = [DXL_LOBYTE(DXL_LOWORD(g2)), DXL_HIBYTE(DXL_LOWORD(g2)), DXL_LOBYTE(DXL_HIWORD(g2)), DXL_HIBYTE(DXL_HIWORD(g2))]
                param_goal_pos_3 = [DXL_LOBYTE(DXL_LOWORD(g3)), DXL_HIBYTE(DXL_LOWORD(g3)), DXL_LOBYTE(DXL_HIWORD(g3)), DXL_HIBYTE(DXL_HIWORD(g3))]

                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL1_ID, param_goal_pos_1)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                    quit()

                # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL2_ID, param_goal_pos_2)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                    quit()

                # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL3_ID, param_goal_pos_3)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
                    quit()

                # Syncwrite goal position
                dxl_comm_result = self.groupSyncWrite.txPacket()

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

                # Clear syncwrite parameter storage
                self.groupSyncWrite.clearParam()

                time.sleep(0.001)

        
    def write_position(self, speed, dxl1_goal_pos, dxl2_goal_pos, dxl3_goal_pos):
        """ 3つのモータを目標位置までゆっくり動かす """
        # Read present position
        dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = self.sync_read_position()

        # Rotate motor slowly
        for i in range(0, 4096):
            if i % speed==0 or i==4095: 
                g1 = int(dxl1_present_pos + (dxl1_goal_pos - dxl1_present_pos) * i / 4095)
                g2 = int(dxl2_present_pos + (dxl2_goal_pos - dxl2_present_pos) * i / 4095)
                g3 = int(dxl3_present_pos + (dxl3_goal_pos - dxl3_present_pos) * i / 4095)

                param_goal_pos_1 = [DXL_LOBYTE(DXL_LOWORD(g1)), DXL_HIBYTE(DXL_LOWORD(g1)), DXL_LOBYTE(DXL_HIWORD(g1)), DXL_HIBYTE(DXL_HIWORD(g1))]
                param_goal_pos_2 = [DXL_LOBYTE(DXL_LOWORD(g2)), DXL_HIBYTE(DXL_LOWORD(g2)), DXL_LOBYTE(DXL_HIWORD(g2)), DXL_HIBYTE(DXL_HIWORD(g2))]
                param_goal_pos_3 = [DXL_LOBYTE(DXL_LOWORD(g3)), DXL_HIBYTE(DXL_LOWORD(g3)), DXL_LOBYTE(DXL_HIWORD(g3)), DXL_HIBYTE(DXL_HIWORD(g3))]

                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL1_ID, param_goal_pos_1)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                    quit()

                # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL2_ID, param_goal_pos_2)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                    quit()

                # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL3_ID, param_goal_pos_3)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
                    quit()

                # Syncwrite goal position
                dxl_comm_result = self.groupSyncWrite.txPacket()

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

                # Clear syncwrite parameter storage
                self.groupSyncWrite.clearParam()

                time.sleep(0.001)

        time.sleep(0.100)
        # Read present position
        dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = self.sync_read_position()
        print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_pos, DXL2_ID, dxl2_present_pos, DXL3_ID, dxl3_present_pos))


    def write_trajectory(self, speed, goal_positions):
        """ 3つのモータを軌道に沿ってゆっくり動かす """
        # Move to start position
        self.angle_initialize(speed, goal_positions[0][0], goal_positions[0][1], goal_positions[0][2])

        #print("press any key to continue!")
        #getch()

        # Rotate motor slowly
        for i in range(len(goal_positions)):
            if i % speed==0 or i==len(goal_positions-1): 
                g1 = goal_positions[i][0]
                g2 = goal_positions[i][1]
                g3 = goal_positions[i][2]

                param_goal_pos_1 = [DXL_LOBYTE(DXL_LOWORD(g1)), DXL_HIBYTE(DXL_LOWORD(g1)), DXL_LOBYTE(DXL_HIWORD(g1)), DXL_HIBYTE(DXL_HIWORD(g1))]
                param_goal_pos_2 = [DXL_LOBYTE(DXL_LOWORD(g2)), DXL_HIBYTE(DXL_LOWORD(g2)), DXL_LOBYTE(DXL_HIWORD(g2)), DXL_HIBYTE(DXL_HIWORD(g2))]
                param_goal_pos_3 = [DXL_LOBYTE(DXL_LOWORD(g3)), DXL_HIBYTE(DXL_LOWORD(g3)), DXL_LOBYTE(DXL_HIWORD(g3)), DXL_HIBYTE(DXL_HIWORD(g3))]

                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL1_ID, param_goal_pos_1)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                    quit()

                # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL2_ID, param_goal_pos_2)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                    quit()

                # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL3_ID, param_goal_pos_3)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
                    quit()

                # Syncwrite goal position
                dxl_comm_result = self.groupSyncWrite.txPacket()

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

                # Clear syncwrite parameter storage
                self.groupSyncWrite.clearParam()

                time.sleep(0.001)

        time.sleep(0.100)

        # Read present position
        dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = self.sync_read_position()
        print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_pos, DXL2_ID, dxl2_present_pos, DXL3_ID, dxl3_present_pos))
