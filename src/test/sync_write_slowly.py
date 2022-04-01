#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library



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

DXL_MINIMUM_POSITION_VALUE  = 0        # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095     # Refer to the Maximum Position Limit of product eManual

#***************************************************




#******************* SETUP *************************

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


# Goal position
dxl_goal_position_1 = [0, 1023, 2047, 3071, 1023]         
dxl_goal_position_2 = [0, 2047, 4095, 3071, 2047]
dxl_goal_position_3 = [0, 3071, 4095, 2047, 1023]
index = 0


# Angle initialize************************************************

print("Angle initializing...")
for i in range(0, 4096):
    if i%20==0 or i==4095: 
        g1 = dxl1_present_position + (dxl_goal_position_1[0] - dxl1_present_position) * i / 4095
        g2 = dxl2_present_position + (dxl_goal_position_2[0] - dxl2_present_position) * i / 4095
        g3 = dxl3_present_position + (dxl_goal_position_3[0] - dxl3_present_position) * i / 4095

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

#**************************************************************************



# Write and read
while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    for i in range(0, 4096):
        if i%20==0 or i==4095: 
            # Allocate goal position value into byte array
            if index == 4:
                g1 = dxl_goal_position_1[4] + (dxl_goal_position_1[0] - dxl_goal_position_1[4]) * i / 4095
                g2 = dxl_goal_position_2[4] + (dxl_goal_position_2[0] - dxl_goal_position_2[4]) * i / 4095
                g3 = dxl_goal_position_3[4] + (dxl_goal_position_3[0] - dxl_goal_position_3[4]) * i / 4095
            else:
                g1 = dxl_goal_position_1[index] + (dxl_goal_position_1[index + 1] - dxl_goal_position_1[index]) * i / 4095
                g2 = dxl_goal_position_2[index] + (dxl_goal_position_2[index + 1] - dxl_goal_position_2[index]) * i / 4095
                g3 = dxl_goal_position_3[index] + (dxl_goal_position_3[index + 1] - dxl_goal_position_3[index]) * i / 4095

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
    #print("hoge")
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

    print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position_1[index + 1], dxl1_present_position, DXL2_ID, dxl_goal_position_2[index + 1], dxl2_present_position, DXL3_ID, dxl_goal_position_3[index + 1], dxl3_present_position))

        

    # Change goal position
    if index != 4:
        index = index + 1
    else:
        index = 0



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
