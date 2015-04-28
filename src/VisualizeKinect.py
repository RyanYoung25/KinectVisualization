#!/usr/bin/env python
# encoding: utf-8


"""Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.

Author: Ryan
ITERO's original project used the Kinect V2 that only ran on windows. This
uses a regular kinect or asus xtion pro and runs entirely on linux.  

The kinect class was authored by: omangin
"""


import roslib
roslib.load_manifest('KinectVisualization')
import rospy
import tf
import signal
import numpy as np
from jsonConverter import jsonMaker
from AngleCalculator import generateAngles
from visualization import SkeletonPlotter
import time

#Inculde sys for command line args
import sys




BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]
LAST = rospy.Duration()

continuing = True




class Kinect:

    def __init__(self, user=1):
        rospy.init_node("kinect_visualizer", anonymous=True)
        self.listener = tf.TransformListener()
        self.user = user

    
    def get_posture(self):
        """Returns a list of frames constituted by a translation matrix
        and a rotation matrix.

        Raises IndexError when a frame can't be found (which happens if
        the requested user is not calibrated).
        """
        try:
            frames = []
            for frame in FRAMES:
                #Block until there are enough tf frames in the buffer
                self.listener.waitForTransform(BASE_FRAME, "/%s_%d" % (frame, self.user), rospy.Time(0), rospy.Duration(4.0))

                trans, rot = self.listener.lookupTransform(BASE_FRAME,"/%s_%d" % (frame, self.user), rospy.Time(0))

                frames.append((frame, trans, rot))
            return frames
        except (tf.LookupException):
            print "User: " + str(self.user) + " not in frame"
        except (tf.ConnectivityException):
            print "Connectivity Exception"
        except (tf.ExtrapolationException):
            print "ExtrapolationException"
        except (tf.Exception):
            print "You done goofed"


def endDemo(signal, frame):
    global continuing
    continuing = False


def main():
    global continuing
    signal.signal(signal.SIGINT, endDemo)
    kinect = Kinect()
    plotter = SkeletonPlotter()
    continuing = True
    while continuing:
        time.sleep(.05)
        value = kinect.get_posture()
        if value is None:
            continue
        jointString = jsonMaker((value,))
        plotter.showSkeleton(jointString, 'b')

def altMain(fileName):
    #run the simulation from a file instead of the kinect
    plotter = SkeletonPlotter()



    try:
        f = open(fileName)
        for line in f:
            plotter.showSkeleton(line, 'b')
        f.close()
    except Exception, e:
        print "File name: " + fileName + " does not exist!"


def MainAngle():
    #run the simulation showing the angles that change
    global continuing
    signal.signal(signal.SIGINT, endDemo)
    kinect = Kinect()
    plotter = SkeletonPlotter()
    continuing = True
    while continuing:
        time.sleep(.05)
        value = kinect.get_posture()
        if value is None:
            continue
        jointString = jsonMaker((value,))
        anglesList = generateAngles(jointString)
        plotter.showSkeletonAngles(jointString, anglesList, 'b')


def showUsage():
    print "Usage: ./VisualizeKinect.py <Visualization Type>"
    print "Types: joints - Shows the skeleton with dots drawn at the joints for the first user"
    print "       angles - Shows the skeleton with dots that change size with the angle made"


if __name__ == '__main__':
    if len(sys.argv) == 2:
        #If the arguements is 2 run one of the simulations
        command = sys.argv[1]
        if command == "joints":
            main()
        elif command == "angles":
            MainAngle()
        else:
            showUsage()

    else:
        showUsage()

 

