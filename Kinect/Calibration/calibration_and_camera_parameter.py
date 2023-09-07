import sys
import os
import cv2
import matplotlib.pyplot as plt
import copy

sys.path.insert(1, "./../")
print(os.getcwd())
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


def my_umeyama(campoints, worldpoints):
    camnormal = campoints - np.mean(campoints, axis=0)
    worldnormal = worldpoints - np.mean(worldpoints, axis=0)
    sig = np.zeros((3, 3))
    sig2 = np.zeros((3, 3))

    for i in range(campoints.shape[0]):
        sig = sig + np.outer(worldnormal[i], camnormal[i])
        sig2 = sig2 + worldnormal[i].reshape(3, 1) @ camnormal[i].reshape(1, 3)

    sig = sig / camnormal.shape[0]
    sig2 = sig2 / camnormal.shape[0]

    det = np.linalg.det(sig)
    print("determinant", det)

    u, d, vt = np.linalg.svd(sig)

    S = np.eye(3)
    if det < 0:
        S[2, 2] = -1
    R = u @ S @ vt

    t = np.mean(worldpoints, axis=0) - R @ np.mean(campoints, axis=0)

    t = t.reshape(1, 3)

    return R, t


bs = []
with open(r"Kinect/Calibration/AuxiliaryFiles/cam1con1.txt") as f:
    for line in f:
        a = line.strip().split(",")
        b = [int(x) for x in a]
        bs.append(b)


f.close()
num = len(bs)

with open(r"Kinect/Calibration/AuxiliaryFiles/cam1con2.txt") as f:
    for line in f:
        a = line.strip().split(",")
        b = [int(x) for x in a]
        bs.append(b)

f.close()
bs = np.array(bs)


XYZ = bs[:, 2:5]
uv = bs[:, 0:2]


pykinect.initialize_libraries()

video1_filename4 = r"Kinect/RecordedVideo/cam1con1.mkv"
playback2 = pykinect.start_playback(video1_filename4)

for i in range(10):
    playback2.update()


playback_config = playback2.get_record_configuration()


depth_images = []

for i in range(5):
    capture1 = playback2.update()
    cal_handle = playback2.get_calibration()

    ret, depth_im = capture1.get_depth_image()
    depth_images.append(depth_im)

depth_images = np.array(depth_images)
depth_image = depth_images.mean(axis=0)

plt.figure()

plt.imshow(depth_image)

plt.show()


uvcon1 = uv[0:num, :]

x_all = []
y_all = []
z_all = []


for i in range(len(uvcon1)):
    ux = uvcon1[i][0]  # column of image pixel
    vy = uvcon1[i][1]  # row of image pixel
    z = depth_image[vy, ux]
    xy = _k4atypes.k4a_float2_t((ux, vy))

    out = cal_handle.convert_2d_to_3d(xy, z, 0, 0)
    x_all.append(out.xyz.x)
    y_all.append(out.xyz.y)
    z_all.append(out.xyz.z)


x_all = np.array(x_all).reshape(len(uvcon1), 1)
y_all = np.array(y_all).reshape(len(uvcon1), 1)
z_all = np.array(z_all).reshape(len(uvcon1), 1)

xyz1 = np.concatenate([x_all, y_all, z_all], axis=1)


playback2.close()


video1_filename4 = r"Kinect/RecordedVideo/cam1con2.mkv"
playback2 = pykinect.start_playback(video1_filename4)

for i in range(10):
    playback2.update()


playback_config = playback2.get_record_configuration()


depth_images = []

for i in range(5):
    capture1 = playback2.update()
    cal_handle = playback2.get_calibration()

    ret, depth_im = capture1.get_depth_image()
    depth_images.append(depth_im)

depth_images = np.array(depth_images)
depth_image = depth_images.mean(axis=0)

plt.figure()

plt.imshow(depth_image)

plt.show()


# FOR CONFIG 2

uvcon2 = uv[num:, :]


x_all = []
y_all = []
z_all = []


for i in range(len(uvcon2)):
    ux = uvcon2[i][0]
    vy = uvcon2[i][1]
    z = depth_image[vy, ux]
    xy = _k4atypes.k4a_float2_t((ux, vy))

    out = cal_handle.convert_2d_to_3d(xy, z, 0, 0)
    x_all.append(out.xyz.x)
    y_all.append(out.xyz.y)
    z_all.append(out.xyz.z)


x_all = np.array(x_all).reshape(len(uvcon2), 1)
y_all = np.array(y_all).reshape(len(uvcon2), 1)
z_all = np.array(z_all).reshape(len(uvcon2), 1)

xyz2 = np.concatenate([x_all, y_all, z_all], axis=1)

xyz = np.vstack((xyz1, xyz2))


XYZ1 = np.zeros(XYZ.shape)
XYZ1[:, :] = XYZ[:, :]


R, t = my_umeyama(xyz, XYZ1)
print(R)
print(t)
p = np.vstack((R, t))

scipy.io.savemat(
    r"Kinect/Calibration/AuxiliaryFiles/new_cam1_params_umeyama.mat", mdict={"name": p}
)

playback2.close()


xyz_converted = xyz @ R.T + t

ax = plt.figure(figsize=(10, 10)).add_subplot(projection="3d")
ax.scatter(XYZ[:, 0], XYZ[:, 2], XYZ[:, 1], marker="o", color="b")
ax.scatter(
    xyz_converted[:, 0], xyz_converted[:, 2], xyz_converted[:, 1], marker="*", color="r"
)
ax.set_xlabel("X Label")
ax.set_ylabel("Y Label")
ax.set_zlabel("Z Label")
plt.show()