## This is Code developed by the CASE Assosiation, built on code and examples
## from Inel Corporation.
## License: MIT See LICENSE file in root directory.
## Modifications Copyright (c) 2020 CASE (Chalmers Autonomous Systems and Electronics)
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

"""
TODO Convert PathPlanner thread to Multiprocessing and include image handling
    get camera init to work
    Implemet CUDA acceleration
    Fix zooming error
    split viewer function into
        Pointcloud handler
        Viewer
"""

import pyrealsense2 as rs
import cv2
import numpy as np
import multiprocessing as mp
from module.ImageHandler import *
from module.Folkracer import Folkracer
from module.PathPlanner import PathPlanner
import time

import signal

frame_width = 424
frame_height = 240
fps = 0
dfps = 90
_camera_car_offset = [0.01, 0,
                      -2.05]  # The offset from camera to car pivot point. [w h l] # todo conv to understandable values?
_car_size = [0.2, 0.1, 0.3, 0.05]  # [w h l r]
blind = 0  # For debugging w/o camera


def rs_init(width=640, height=480, fps=0, dfps=0):
    """
    Setup function for Realsens pipe
    :param width: of camera stream
    :param height: of camera stream
    :param fps: of camera stream
    :return: created pipeline
    """

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    if dfps > 0:
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, dfps)
    if fps > 0:
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    # Start streaming
    pipeline.start(config)

    return pipeline


def disp(camera, run=True):
    theta = 0  # Scanning direction, 0 = straight ahead
    dtheta = 5  # Scanning step in degrees
    x0 = [0, 0]
    d = 1.45

    r = 5
    #  scale = camera.scale  # todo imp scaling, now 1000:1
    x_range = [-1, 1]
    y_range = [-0.1,
               0.1]  # height span in meters  # todo dynamic y_range ~ depth according to height over "driving plane"
    z_range = [0.01, 5]  # depth span in meters
    theta_range = 60
    _t_last = 0
    _fps = 30
    _plot_scale = frame_height / z_range[1]
    _scan_dir = 1

    def line(theta, r, x0):
        """
        calc vector from polar params
        :param theta: angle
        :param r: vector length
        :param x0: vector origin
        :return: vector
        """
        R = r * _plot_scale
        x1 = int(x0[0] * _plot_scale + frame_width / 2)
        y1 = int(x0[1] * _plot_scale + frame_height)
        x2 = int(x1 + R * np.sin(theta))
        y2 = int(y1 - R * np.cos(theta))
        return (x1, y1), (x2, y2)

    def delta_x(ang, d):
        return (d * np.cos(ang)) / 2

    def inliers(points, theta, d, x0):
        """
        Check if points is in between the two parallel lines along the vector
        from x0 and  with angle theta.
        :param points: inlier candidate
        :param theta: angle of vector, 0 = straight ahead
        :param d: distance between parallel lines, width of car + margin
        :param x0: car center point
        :return: inliers [x, y]^T
        """
        X, Z = points[0], points[1]
        k = 1 / np.tan(theta)
        # mlim = np.add(x0[0], [-d/2, d/2])  # upper and lover lim of m
        mlim = [-d / 2, d / 2]
        m = np.subtract(np.dot(k, X), Z)  # m = kX-Y
        # Inlier indexes
        I = np.where(np.logical_and(mlim[0] <= m, mlim[1] >= m))
        return [X[I], Z[I]]

    def plot_pointcloud(points, only_closes=False, step=3):
        pts = np.multiply(points, _plot_scale)
        pts[:, 0] += frame_width / 2  # center x axis
        pts[:, 2] = frame_height - pts[:, 2]  # Flip z projection
        pts = pts.astype(int)
        # pts[:, 1] *= 255/np.max(np.abs(pts[:, 1]))  # scale to fit grayscale
        # _inliers = np.where(np.logical_and(pts[0] <= frame_width, pts[2] <= frame_height))
        # pts[1] -= frame_height
        # pts_coord = np.transpose([pts[:, 0], pts[:, 2], pts[:, 1]])
        gray = np.zeros((frame_height, frame_width), np.uint8)

        if only_closes:
            for w in range(0, frame_width, step):
                i = np.where(np.logical_and(pts[:, 0] >= w, pts[:, 0] < w + step))[0]  # get points in line w
                if i.any():
                    j = np.where(pts[i, 2] == np.min(pts[i, 2]))[0]  # get closest point on line w
                    k = i[j[0]]
                    p = pts[k]
                    if 0 <= p[0] < frame_width and 0 <= p[2] < frame_height and p[1] <= 255:
                        gray[p[2], p[0]] = p[1]
        else:
            for p in pts.astype(int):
                if 0 <= p[0] < frame_width and 0 <= p[2] < frame_height and p[1] <= 255:
                    gray[p[2], p[0]] = p[1]

        return cv2.applyColorMap(gray, cv2.COLORMAP_JET)

    window_name = ('Depth scan')
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    l_thickness = 2
    plot = np.ones((frame_height, frame_width, 3), np.uint8) * 215

    while run:
        try:
            points = camera.get_verts()
            _t_now = time.perf_counter()
            hz = np.round(1 / (_t_now - _t_last), 2)
            _t_last = _t_now

            # plot = np.ones((frame_height, frame_width, 3), np.uint8) * 215  # clear frame
            plot = plot_pointcloud(points, 1, 3)

            X, Y, Z = points[:, 0], points[:, 1], points[:, 2]
            y_inrange = np.where(np.logical_and(Y >= y_range[0], Y <= y_range[1]))
            z_inrange = np.where(np.logical_and(Z >= z_range[0], Z <= z_range[1]))
            pts_inrange = np.intersect1d(y_inrange, z_inrange)
            px, py = X[pts_inrange], Z[pts_inrange]

            if abs(theta) > np.deg2rad(theta_range / 2):
                theta = np.deg2rad(-theta_range / 2)
            else:
                theta += np.deg2rad(dtheta)

            pxi, pyi = inliers([X[pts_inrange], Z[pts_inrange]], theta, d, x0)

            dx = delta_x(theta, d)
            ll = line(theta, r - 0.1, np.subtract(x0, [dx, 0]))
            lc = line(theta, r, x0)
            lr = line(theta, r - 0.1, np.add(x0, [dx, 0]))

            """plot = cv2.line(plot, ll[0], ll[1], (0, 0, 255), l_thickness)
            plot = cv2.line(plot, lc[0], lc[1], (0, 255, 0), l_thickness)
            plot = cv2.line(plot, lr[0], lr[1], (0, 0, 255), l_thickness)"""

            cv2.imshow(window_name, plot)
            _fps = round(0.99 * _fps + 0.01 * hz, 1)  # some smoothening
            window_title = ('Depth Scan | Theta: {:5}deg | Fps: {:5.1f}Hz'.format(np.round(np.rad2deg(theta)), _fps))
            cv2.setWindowTitle(window_name, window_title)
            time.sleep(0.001)

            if cv2.waitKey(1) in (27, ord("q")):
                run = False  # esc to quit

            print(_fps)



        except KeyboardInterrupt:
            run = False


def plot_pointcloud(points, only_closes=False, step=3):
    z_range = [0.01, 5]  # depth span in meters
    _plot_scale = frame_height / z_range[1]

    pts = np.multiply(points, _plot_scale)
    pts[:, 0] += frame_width / 2  # center x axis
    pts[:, 2] = frame_height - pts[:, 2]  # Flip z projection
    pts = pts.astype(int)
    # pts[:, 1] *= 255/np.max(np.abs(pts[:, 1]))  # scale to fit grayscale
    # _inliers = np.where(np.logical_and(pts[0] <= frame_width, pts[2] <= frame_height))
    # pts[1] -= frame_height
    # pts_coord = np.transpose([pts[:, 0], pts[:, 2], pts[:, 1]])
    gray = np.zeros((frame_height, frame_width), np.uint8)

    if only_closes:
        for w in range(0, frame_width, step):
            i = np.where(np.logical_and(pts[:, 0] >= w, pts[:, 0] < w + step))[0]  # get points in line w
            if i.any():
                j = np.where(pts[i, 2] == np.min(pts[i, 2]))[0]  # get closest point on line w
                k = i[j[0]]
                p = pts[k]
                if 0 <= p[0] < frame_width and 0 <= p[2] < frame_height and p[1] <= 255:
                    gray[p[2], p[0]] = p[1]
    else:
        for p in pts.astype(int):
            if 0 <= p[0] < frame_width and 0 <= p[2] < frame_height and p[1] <= 255:
                gray[p[2], p[0]] = p[1]

    return cv2.applyColorMap(gray, cv2.COLORMAP_JET)


def main():
    """
    Main Folkrace function
    :return: None
    """
    # initialize Realsense Camera
    camera = Camera(rs_init(frame_width, frame_height, fps, dfps))

    # Creat Car object and Path planner

    #   Car is the process handling the dynamics of the car
    Car = Folkracer(car_size=_car_size, camera_car_offset=_camera_car_offset)
    pp = PathPlanner(Camera=camera)

    # Start threads
    # Car.start()
    # pp.start()

    hz = 0
    _fps = 30
    _theta = 0.0
    _phi = 0.0
    theta = []
    run = True
    _t_last_print = 0
    try:
        dist = 0
        while run:
            _t_start = time.perf_counter()

            tunnel_size = Car.size[0:2]
            verts = camera.get_verts()

            _max_dist = 0
            for _theta in range(-25, 26, 5):

                inliers = pp.process_verts(verts, tunnel_size=tunnel_size, theta=_theta, phi=_phi)

                if inliers.any():
                    Z = inliers[:, 2]
                    _dist = np.min(Z)
                    if _dist > _max_dist:
                        theta = _theta
                        _max_dist = _dist

            _t_fin = time.perf_counter()

            if _t_last_print + .05 < time.perf_counter():
                _t_last_print = time.perf_counter()
                _dt = _t_fin - _t_start
                hz = 1/_dt
                _fps = (0.9 * _fps + 0.1 * hz)
                dist = (0.9 * dist + 0.1 * _max_dist)
                print('Dist {}m | Theta  {}deg |  f: {}Hz'.format(round(dist, 2), theta, round(_fps, 2)))



    except KeyboardInterrupt:
        pass
    # Finish, close Car handler, Path planner and stream
    # Car.end()
    pp.end()
    camera.end()
    print('Main process have ended')
    exit(0)


if __name__ == '__main__':
    main()
