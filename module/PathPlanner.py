import pyrealsense2 as rs
import numpy as np
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

        self.points = self.process_cloud()
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

    def process_cloud(self):
        """
        Process pointcloud from Camera
        :return:
        """

        #v, t, verts, texcoords, color_source, mapped_frame = self.cam.get_data()

        verts = self.cam.get_verts()

        return np.transpose(verts)

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
