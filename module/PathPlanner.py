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
import multiprocessing as mp
from module.ImageHandler import *


# todo migrate from multithread to multiprocessing
class PathPlanner:
    def __init__(self, Camera, Car):
        """
        Gradient decent solving path planner ## todo Evaluate against other methods of path planning
        :param Car: Object to give driving vector
        """
        self.cam = Camera
        self.cam.decimate.set_option(rs.option.filter_magnitude, 2 ** 3)
        self.car = Car
        self.running = False

        self.points = np.transpose(self.cam.get_verts())  # todo fix some function to call instead
        self.ground_plane = self._get_ground_plane()

        # Direction variables
        self.theta = 0.0
        self.phi = 0.0
        self.dist = -1.0
        self.last_reading = 0.0
        self.samplerate = 0.0
        self.fov = 50
        self.q = mp.Queue()

        self.pp = None

        #self.path_vector = self.get_path()

    def start(self):
        self.running = True
        #path_plan = mt.Thread(target=self.path_plan, name='path_planner')
        self.pp = mp.Process(target=self.path_plan, name='path_planner', args=(self.q,))
        try:
           self.pp.start()
        except Exception as err:
            print("Error: unable to start path_planner thread, err: {}".format(err))
        return self.pp

    def end(self):
        if self.running:
            self.q.put("END")
            self.pp.join(timeout=1)
            self.q.close()
            self.q.join_thread()
            if self.running:
                self.pp.terminate()
                print('\n\033[91m Path planner terminated\033[0m')
            else:
                print('\n\033[92m Path planner ended\033[0m')


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

    def _get_path(self):
        theta = None
        tunnel_size = self.car.size[0:2]
        verts = self.cam.get_verts()

        _max_dist = 0
        for _theta in range(int(self.theta - self.fov/2), int(self.theta + self.fov/2 + 1), 5):

            inliers = self.process_verts(verts, tunnel_size=tunnel_size, theta=_theta, phi=self.phi)

            if inliers.any():
                Z = inliers[:, 2]
                _dist = np.min(Z)
                if _dist > _max_dist * 1.1:  # Must be more than 10 grater
                    theta = _theta
                    _max_dist = _dist

        if theta:
            return _max_dist, theta
        return 0.0, theta

    def path_plan(self, q):
        rx_msg = []
        while self.running:
            """try:
                rx_msg = q.get(block=False)
                if "END" in rx_msg:
                    self.running = False
                    return
            except Exception as err:
                pass
                #print(err)"""
            self.ground_plane = self._get_ground_plane()
            #self.dist, self.theta = self._get_path()
            _t_now = time.perf_counter()
            self.samplerate = 1 / (_t_now - self.last_reading)
            self.last_reading = _t_now
            tx_msg = {'dist': self.dist, 'rate': self.samplerate, 'theta': self.theta}
            q.put(tx_msg)  # send data as Queue message




