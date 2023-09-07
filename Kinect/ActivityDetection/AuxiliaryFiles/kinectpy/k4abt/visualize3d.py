import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinectpy.k4abt._k4abtTypes import K4ABT_JOINT_COUNT, K4ABT_SEGMENT_PAIRS, body_colors
from kinectpy.k4abt.joint import Joint
from kinectpy.k4abt.body import Body

class SkeletonPlotter:

    def __init__(self):
        #make the figure, set the axis
        self.fig = plt.figure(figsize=(16, 20))
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

    def visualize(self, bodyFrame, only_segments = False):
		
        color = (int (body_colors[bodyFrame.id][0]), int (body_colors[bodyFrame.id][1]), int (body_colors[bodyFrame.id][2]))

        for segmentId in range(len(K4ABT_SEGMENT_PAIRS)):
            segment_pair = K4ABT_SEGMENT_PAIRS[segmentId]
            point1 = bodyFrame.joints[segment_pair[0]].get_coordinates()
            point2 = bodyFrame.joints[segment_pair[1]].get_coordinates()

            if (point1[0] == 0 and point1[1] == 0) or (point2[0] == 0 and point2[1] == 0):
                continue

        image = cv2.line(image, point1, point2, color, 2)

        if only_segments:
            return image

        for joint in bodyFrame.joints:
            image = cv2.circle(image, joint.get_coordinates(), 3, color, 3)

        return image

