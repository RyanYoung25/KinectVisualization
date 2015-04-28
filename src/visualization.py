#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import json

class SkeletonPlotter:

    def __init__(self):
        #make the figure, set the axis
        self.fig = plt.figure(figsize=(15,15))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.view_init(elev=10., azim=210)
        self.ax.set_autoscale_on(False)

        #Set scales
        self.ax.set_xlim(0, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-2, 2)

        #Set the max size the angle dots can get
        self.maxSize = 50


        self.oldPoints = None
        self.oldLines = []
        #Make the plotting interactive, Show the plot
        plt.ion() 
        plt.show()


    def setup(self):
        #Make the plotting no longer interative
        plt.ion()

    def showSkeleton(self, jointString, color):
        '''
        Takes a list containing joints for a user. 
        Displays the skeleton in 3d based off of the joint data
        passed in.

        The index of joints:

        JOINTS = [
           0 'head',
           1 'neck',
           2 'torso',
           3 'left_shoulder',
           4 'left_elbow',
           5 'left_hand',
           6 'left_hip',
           7 'left_knee',
           8 'left_foot',
           9 'right_shoulder',
           10 'right_elbow',
           11 'right_hand',
           12 'right_hip',
           13 'right_knee',
           14 'right_foot'
            ]
        '''
        jointList = json.loads(jointString)["Joints"]
        #Make the points
        x = []
        y = []
        z = []

        for joint in jointList:
            x.append(joint['pos']['x'])
            y.append(joint['pos']['y'])
            z.append(joint['pos']['z'])

        #Remove the old points if they need to be removed
        if self.oldPoints != None:
            self.oldPoints.remove()
            for line in self.ax.lines:
                line.remove()


        #Plot the point
        self.oldPoints = self.ax.scatter(x, y, z, c=color, marker='o')

        #Make the lines
        #Make an array of tuples where the first element in 
        #the tuple is the first joint number and second element is the second. 
        lineList = [  (0, 1), #Head to neck
                      (3, 1), #Left shoulder to neck
                      (9, 1), #Right shoulder to neck
                      (3, 4), 
                      (4, 5),
                      (6, 3), 
                      (6, 7), 
                      (7, 8), 
                      (9, 10), 
                      (10, 11), 
                      (12, 9), 
                      (12, 13), 
                      (13, 14),
                      (2, 3),
                      (2, 6),
                      (2, 9),
                      (2, 12)
                      ]

        #Draw each line 
        for ind1, ind2 in lineList:
          self.ax.plot([x[ind1],x[ind2]], [y[ind1], y[ind2]], [z[ind1], z[ind2]], color= color)

        plt.draw()


    def showSkeletonAngles(self, jointString, angleList, color):
            '''
            Takes a list containing joints for a user. 
            Displays the skeleton in 3d based off of the joint data
            passed in.

            The index of joints:

            JOINTS = [
               0 'head',
               1 'neck',
               2 'torso',
               3 'left_shoulder',
               4 'left_elbow',
               5 'left_hand',
               6 'left_hip',
               7 'left_knee',
               8 'left_foot',
               9 'right_shoulder',
               10 'right_elbow',
               11 'right_hand',
               12 'right_hip',
               13 'right_knee',
               14 'right_foot'
                ]
            '''
            jointList = json.loads(jointString)["Joints"]
            #Make the points
            x = []
            y = []
            z = []
            size = []

            for joint in jointList:
                x.append(joint['pos']['x'])
                y.append(joint['pos']['y'])
                z.append(joint['pos']['z'])
                size.append(1)
                if joint['name'] == 'right_elbow':
                    size[-1] = self.maxSize * (2 ** math.fabs(angleList[0] - 3.14))
                    print math.fabs(angleList[0])
                elif joint['name'] == 'left_elbow':
                    size[-1] = self.maxSize * (2 ** math.fabs(angleList[1] - 3.14))
                elif joint['name'] == 'left_shoulder':
                    size[-1] = self.maxSize * (2 ** math.fabs(angleList[5] + .2))
                elif joint['name'] == 'right_shoulder':
                    size[-1] = self.maxSize * (2 ** math.fabs(angleList[4] - .2))


            #Make angle adust size

            #Remove the old points if they need to be removed
            if self.oldPoints != None:
                self.oldPoints.remove()
                for line in self.ax.lines:
                    line.remove()


            #Plot the point
            self.oldPoints = self.ax.scatter(x, y, z, c=color, marker='o', s=size)

            #Make the lines
            #Make an array of tuples where the first element in 
            #the tuple is the first joint number and second element is the second. 
            lineList = [  (0, 1), #Head to neck
                          (3, 1), #Left shoulder to neck
                          (9, 1), #Right shoulder to neck
                          (3, 4), 
                          (4, 5),
                          (6, 3), 
                          (6, 7), 
                          (7, 8), 
                          (9, 10), 
                          (10, 11), 
                          (12, 9), 
                          (12, 13), 
                          (13, 14),
                          (2, 3),
                          (2, 6),
                          (2, 9),
                          (2, 12)
                          ]

            #Draw each line 
            for ind1, ind2 in lineList:
              self.ax.plot([x[ind1],x[ind2]], [y[ind1], y[ind2]], [z[ind1], z[ind2]], color= color)

            plt.draw()




