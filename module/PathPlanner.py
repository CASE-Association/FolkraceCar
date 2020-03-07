from typing import Union

import pyrealsense2 as rs
from cupy.core.core import ndarray
from numpy.core._multiarray_umath import ndarray

"""_CUDA = False
try:
    import cupy as cp
    _CUDA = True
    print('Using CuPy for CUDA acceleration! :D')
except Exception as err:
    print(err)
    import numpy as cp
    print('CuPy unsuported, Using Numpy as cp replacement :(')"""

from numpy.linalg import svd, det
import time
import threading as mt
from module.ImageHandler import *


# todo migrate from multithread to multiprocessing
class PathPlanner:
    def __init__(self, Camera):
        """
        Gradient decent solving path planner ## todo Evaluate against other methods of path planning
        :param Car: Object to give driving vector
        """
        self.cam = Camera
        self.running = False

        self.points = np.transpose(self.cam.get_verts())  # todo fix some function to call instead
        self.ground_plane = self._get_ground_plane()

        self.path_vector = self.get_path()

    def start(self):
        self.running = True
        path_plan = mt.Thread(target=self.path_plan, name='path_planner')
        try:
            path_plan.start()
        except Exception as err:
            print("Error: unable to start path_planner thread, err: {}".format(err))
        return path_plan

    def end(self):
        self.running = False

    def _rigid_transform_3D(self, A, B):
            assert len(A) == len(B)

            N = A.shape[0]  # total points

            centroid_A = np.mean(A, axis=0)
            centroid_B = np.mean(B, axis=0)

            # centre the points
            AA = A - np.tile(centroid_A, (N, 1))
            BB = B - np.tile(centroid_B, (N, 1))

            # dot is matrix multiplication for array
            H = np.transpose(AA) * BB

            U, S, Vt = svd(H)

            R = Vt.T * U.T

            # special reflection case
            if det(R) < 0:
                Vt[2, :] *= -1
                R = Vt.T * U.T

            t = -R * centroid_A.T + centroid_B.T

            return R, t

    def _get_ground_plane(self):
        """
        Estimate ground plain using Least square
        todo evaluate usage of RANSAC
        :return: groudn plain origin and rotation
        """

        pos, normal = self.planeFit()

        rot, _ = self._rigid_transform_3D(pos, normal)

        rot = np.eye(3)  # todo calculate rotation matrix

        return pos, rot, normal

    def planeFit(self):
        """
        Original code from: https://stackoverflow.com/a/18968498

        p, n = planeFit(points)

        Given an array, points, of shape (d,...)
        representing points in d-dimensional space,
        fit an d-dimensional plane to the points.

        :return: a point, p, on the plane (the point-cloud centroid),
        and the normal, n.
        """
        points = self.points

        points = np.reshape(points, (np.shape(points)[0], -1))  # Collapse trialing dimensions
        assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1],
                                                                                                       points.shape[0])
        ctr = points.mean(axis=1)
        x = points - ctr[:, np.newaxis]
        M = np.dot(x, x.T)  # Could also use np.cov(x) here.
        return ctr, svd(M)[0][:, -1]

    def process_verts(self, verts, tunnel_size, theta=0.0, phi=0.0, ground_plane=None):
        """
        Process verts from Camera.

        :param tunnel_size: the desired tunnel size, [w, h]
        :param theta: The desired turning angle
        :param phi: The desired incline angle.
        :param ground_plane: The estimated driving plane.
        :return: All points in defined direction and for the defined tunnel size.
        """
        # todo
        #  implement as cp
        #  driving plane

        #p = cp.asanyarray(verts)
        p = verts

        X, Y, Z = p[:, 0], p[:, 1], p[:, 2]
        [w, h] = tunnel_size

        # inlier index
        I = []

        """
        Basic depth filter (Z)
        do not allow points with no distance (z=0)
        """

        I = np.where((Z > 0.0))[0]
        # if no inliers return None
        if not I.any():
            return None

        """
        Filter height (Y) inliers    # todo implement plane filter instead of static height
        
        using the line equation y = kx + m we can  formulate the inlier constraint as
        -mlim <= m <= mlim for m = y - kx. 
        In this case we substitute y with the Y-array (Height) and x with the Z - array (Depth).  
        """
        radPhi = np.deg2rad(phi)
        k_y = np.tan(-radPhi)  # negate to get correct incline direction
        mlim_h = [-h / 2, h / 2]
        m = np.subtract(Y[I], np.multiply(k_y, Z[I]))  # {m = y - kx} => m = Y - k.*Z
        # Inlier indexes
        I_y = np.where(np.logical_and(mlim_h[0] <= m, mlim_h[1] >= m))[0]

        I = I[I_y]

        """
        Filter width (X) inliers 
        
        using the same equation as above but now with the m being a function of k,X and Z.
        we now only use inliers (I_y) form the above step to reduce necessary computations.
        """
        radTheta = np.deg2rad(theta)
        k_x = np.tan(radTheta)
        mlim_w = [-w / 2, w / 2]
        m = np.subtract(X[I], np.multiply(k_x, Z[I]))  # {m = y - kx} => m = X - k.*Z
        # Inlier of Y inliers
        I_x = np.where(np.logical_and(mlim_w[0] <= m, mlim_w[1] >= m))[0]

        # Pop outliers from I
        I = I[I_x]


        return p[I]

    def get_path(self):
        pass
        '''
        h = [0, 100]  # Heightspan of interset

        for p in self.points:
            if h[0] <= p[1] <= h[1]:

        self.path_vector = [0, 0]        
        '''
        return [0, 0]


    def path_plan(self):
        while self.running:
            self.points = self.process_cloud()
            self.ground_plane = self._get_ground_plane()
            time.sleep(0.1)

        print('Path planner ended')