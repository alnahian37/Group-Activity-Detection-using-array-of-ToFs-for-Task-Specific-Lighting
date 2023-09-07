import sys
import os
import cv2
import matplotlib.pyplot as plt

sys.path.insert(1, "./../")
sys.path.insert(
    0, os.path.join(os.getcwd(), r"Kinect\\ActivityDetection\\AuxiliaryFiles")
)

import kinectpy as pykinect
import numpy as np

from kinectpy.k4a import _k4a
from kinectpy.k4a.capture import Capture
from kinectpy.k4a.transformation import Transformation
import scipy.io

import json
import time
import pickle

device_config = pykinect.default_configuration
from kinectpy.k4a import _k4atypes

if __name__ == "__main__":
    pykinect.initialize_libraries()

    for i in range(1, 3):
        video1_filename = (r"Kinect/RecordedVideo/cam1con" + str(i) + ".mkv")  # Load video here
        print(video1_filename)

        playback1 = pykinect.start_playback(video1_filename)
        for j in range(10):
            playback1.update()

        capture1 = playback1.update()

        playback_calibration1 = playback1.get_calibration()
        playback_config1 = playback1.get_record_configuration()

        print(capture1)
        a = Capture.get_color_image(capture1)
        print(a[0])
        b = Capture.get_depth_image(capture1)

        c = Capture.get_transformed_depth_image(capture1)

        _, d = capture1.get_ir_image()

        filename = r"Kinect/Calibration/AuxiliaryFiles/rgb/cam1image" + str(i) + ".jpg"
        cv2.imwrite(filename, a[1])

        # filename=r"D:\SCULPT\Nahian\Video collected\check_images\depth\mas"+str(i)+".jpg"
        filename = (r"Kinect/Calibration/AuxiliaryFiles/depth/cam1image" + str(i) + ".mat")
        scipy.io.savemat(filename, mdict={"name": b[1]})

        # filename=r"D:\SCULPT\Nahian\Video collected\check_images\modified depth\mas"+str(i)+".jpg"
        filename = (r"Kinect/Calibration/AuxiliaryFiles/modified depth/cam1image"+ str(i)+ ".mat")
        # cv2.imwrite(filename, c[1])
        scipy.io.savemat(filename, mdict={"name": c[1]})

        filename = (r"Kinect/Calibration/AuxiliaryFiles/ir images/cam1image" + str(i) + ".mat")
        # cv2.imwrite(filename, c[1])
        scipy.io.savemat(filename, mdict={"name": d})
        playback1.close()
