#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from robot_model import *
from motor_driver import *



class TrajectoryFunc:

    def __init__(self):
        # Robot model
        self.model = RobotModel()


    def xy_circle(self, center, radius, direction="l"):
        """ XY平面上の円軌道のモータ角を出す """
        angle = np.zeros((4096, 3), dtype=int)
        position = [0, 0, 0]

        deg = range(4096)

        if direction == "r":
            deg.reverse()
        elif direction != "l":
            print("Direction is invalid!!")
            quit()
            

        for i in deg:
            # x = rcos(i), y = rsin(i)
            position[0] = center[0] + radius * np.cos((deg[i] / 4096.0) * 2 * np.pi)
            position[1] = center[1] + radius * np.sin((deg[i] / 4096.0) * 2 * np.pi)
            position[2] = center[2]
            # Solve IK
            angle[i] = 2048 + self.model.solve_ik(position) * 2048 / np.pi

            for j in range(3):
                if angle[i][j] > DXL_MINIMUM_POSITION_VALUE and angle[i][j] < DXL_MAXIMUM_POSITION_VALUE:
                    continue
                else:
                    print("Target[%d] : Motor angle [%d] is out of range!! (%03d)" % (i+1, j+1, angle[i][j]))
                    quit()

        return angle
            




