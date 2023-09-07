import numpy as np
import cv2

from kinectpy.k4abt.joint import Joint
from kinectpy.k4abt._k4abtTypes import K4ABT_JOINT_COUNT, K4ABT_SEGMENT_PAIRS
from kinectpy.k4abt._k4abtTypes import k4abt_skeleton_t, k4abt_body_t, body_colors
from kinectpy.k4a._k4atypes import K4A_CALIBRATION_TYPE_DEPTH


height = 720
width = 1280

Gx = 0
Gy = 0
Gz = 0

RotX = 0
RotY = 0
RotZ = 0

Cx = 0
Cy = 0
Cz = 5

f = 0.1
Px = 0.06
Py = 0.048
offsetX = width / 2
offsetY = height / 2
skew = 0


class Body3d:
    
    def __init__(self, body3d_handle):

        if body3d_handle:
            self._handle = body3d_handle
            self.id = body3d_handle.id
            self.initialize_skeleton()


    def __del__(self):
        self.destroy()


    def is_valid(self):
        return self._handle


    def handle(self):
        return self._handle


    def destroy(self):
        if self.is_valid():
            self._handle = None


    def initialize_skeleton(self):
        joints = np.ndarray((K4ABT_JOINT_COUNT,), dtype=np.object)

        for i in range(K4ABT_JOINT_COUNT):
            joints[i] = Joint(self._handle.skeleton.joints[i], i)

        self.joints = joints


    def get_joint_positions(self):
        all_x = []
        all_y = []
        all_z = []
        joints = self.handle().skeleton.joints
        for i in range(K4ABT_JOINT_COUNT):
            x, y, z = self.joints[i].get_coordinates()
            all_x.append(x)
            all_y.append(y)
            all_z.append(z)
        all_x = np.array(all_x)
        all_y = np.array(all_y)
        all_z = np.array(all_z)
        joints_position = np.transpose(np.vstack([all_x, all_y, all_z]))
        # homogenize
        #return np.hstack((joints_position, np.ones((len(joints), 1))))
        return np.transpose(joints_position)

    def get_camera(self):
        offset = np.array([[1, 0, 0, offsetX],
                        [0, -1, 0, offsetY],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        P = np.array([[(f * width) / (2 * Px), skew, 0, 0],
                    [0, (f * height) / (2 * Py), 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])

        C = np.array([[1, 0, 0, -Cx],
                    [0, 1, 0, -Cy],
                    [0, 0, 1, -Cz],
                    [0, 0, 0, 1]])

        Rx = np.array([[1, 0, 0, 0],
                    [0, np.cos(RotX), - np.sin(RotX), 0],
                    [0, np.sin(RotX), np.cos(RotX), 0],
                    [0, 0, 0, 1]])

        Ry = np.array([[np.cos(RotY), 0, np.sin(RotY), 0],
                    [0, 1, 0, 0],
                    [- np.sin(RotY), 0, np.cos(RotY), 0],
                    [0, 0, 0, 1]])

        Rz = np.array([[np.cos(RotZ), - np.sin(RotZ), 0, 0],
                    [np.sin(RotZ), np.cos(RotZ), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        G = np.array([[1, 0, 0, -Gx],
                    [0, 1, 0, -Gy],
                    [0, 0, 1, -Gz],
                    [0, 0, 0, 1]])
        
        T = np.array([[-9.28595566],
                      [-0.4060233 ],
                      [ 3.34561988],
                      [ 0.        ]])

        T = np.reshape(T, (4,))

        joint_positions_h = np.transpose(self.get_joint_positions())
        joint_positions_h = np.hstack((joint_positions_h, np.ones((len(joint_positions_h), 1))))
        x = [0] * len(joint_positions_h)

        for i in range(len(joint_positions_h)):
            x[i] = np.matmul(G, np.array(joint_positions_h[i]))
            x[i] = np.matmul(Rz, x[i])
            x[i] = np.matmul(Ry, x[i])
            x[i] = np.matmul(Rx, x[i])
            x[i] = np.matmul(C, x[i])
            x[i] = np.matmul(P, x[i])
            N = np.array([[1 / x[i][2], 0, 0, 0],
                        [0, 1 / x[i][2], 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

            x[i] = np.matmul(N, x[i])
            x[i] = np.matmul(offset, x[i])
        
        return x


    def render(self, canvas):

        #color = (int (body_colors[self.handle().id][0]), int (body_colors[self.handle().id][1]), int (body_colors[self.handle().id][2]))
        camera = self.get_camera()
        for segmentId in range(len(K4ABT_SEGMENT_PAIRS)):
            segment_pair = K4ABT_SEGMENT_PAIRS[segmentId]
            cv2.line(canvas,
                    (int(camera[segment_pair[0]][0]), int(camera[segment_pair[0]][1])),
                    (int(camera[segment_pair[1]][0]), int(camera[segment_pair[1]][1])),
                    color = (143, 143, 8) ,
                    thickness=5)
        
        return canvas


    @staticmethod
    def create(body_handle, calibration, bodyIdx, dest_camera):

        skeleton_handle = k4abt_skeleton_t()
        body3d_handle = k4abt_body_t()

        for jointID, joint in enumerate(body_handle.skeleton.joints): 
            skeleton_handle.joints[jointID].position = joint.position
            skeleton_handle.joints[jointID].confidence_level = joint.confidence_level

        body3d_handle.skeleton = skeleton_handle
        body3d_handle.id = bodyIdx

        return Body3d(body3d_handle)


    def __str__(self):
        """Print the current settings and a short explanation"""
        message = f"Body Id: {self.id}\n\n"

        for joint in self.joints:
            message += str(joint)

        return message

