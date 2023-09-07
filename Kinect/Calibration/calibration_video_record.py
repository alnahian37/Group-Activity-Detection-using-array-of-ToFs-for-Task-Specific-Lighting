import os
import sys

sys.path.insert(1, "./../")

print(os.getcwd())
sys.path.insert(
    0, os.path.join(os.getcwd(), r"Kinect\\ActivityDetection\\AuxiliaryFiles")
)
import json
import time
import pickle
import ast
import gc
import scipy.io
import numpy as np
import csv
import copy
from scipy.optimize import linear_sum_assignment
import csv
import pickle
import json
import ast
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import inf
from timeit import repeat
import kinectpy as kpy
from kinectpy.k4abt._k4abtTypes import K4ABT_SEGMENT_PAIRS
from kinectpy.k4abt import _k4abt
from kinectpy.k4a import _k4a
from kinectpy.k4a._k4atypes import (
    K4A_CALIBRATION_TYPE_DEPTH,
    K4A_CALIBRATION_TYPE_COLOR,
)
import logging
from matplotlib import animation
from datetime import datetime
import socket
from _thread import *


gc.enable()

kpy.initialize_libraries()

fps = int(input("Enter the fps: 0 for 5fps, 1 for 15fps, 2 for 30fps: "))

device_number = int(input("Enter camera number (1,2,3,4): "))

config_number = input("Enter config number (1 or 2): ")

# Start the devices
device_config = copy.deepcopy(kpy.default_configuration)
device_config.color_resolution = 1  # 0:off, 1:720, 2:1080, 3:1440, 4:1536, 5:2160p
device_config.depth_mode = 2  # (0:OFF, 1:NFOV_2X2BINNED, 2:NFOV_UNBINNED,3:WFOV_2X2BINNED, 4:WFOV_UNBINNED, 5:Passive IR)
device_config.wired_sync_mode = (
    0  # (0:Standalone mode, 1:Master mode, 2:Subordinate mode)
)
device_config.camera_fps = fps


# Modify Here

single_file_time_min = int(input("Enter single video length (in seconds): "))
single_file_time = single_file_time_min  # Seconds


total_file = 1  # Total number of files to be recorded


i = 0

video_filename4 = (
    r"Kinect/RecordedVideo/cam" + str(device_number) + "con" + config_number + ".mkv"
)


playback4 = kpy.start_device(
    device_index=device_number - 1,
    config=device_config,
    record=True,
    record_filepath=video_filename4,
)


t1 = time.time()

t_loop_start = time.time()
start_flag = 1

while time.time() - t1 < single_file_time:
    playback4.update()

    if start_flag:
        print("Recording Started")
        start_flag = 0

playback4.close()

gc.collect()

print("Finished Recording...")