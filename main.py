## This is Code developed by the CASE Assosiation, built on code and examples
## from Inel Corporation.
## License: MIT See LICENSE file in root directory.
## Modifications Copyright (c) 2020 CASE (Chalmers Autonomous Systems and Electronics)
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

"""
TODO Convert PathPlanner thread to Multiprocessing and include image handling
"""

'''
TODO
    Fix zooming error
    split viewer function into
        Pointcloud handler
        Viewer
'''
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import time
from module.MatrixMagic import *
from module.Folkracer import Folkracer
from module.PathPlanner import PathPlanner


frame_width = 640
frame_height = 480
fps = 30
_camera_car_offset = [0.01, 0, -2.05]  # The offset from camera to car pivot point. [w h l] # todo conv to understandable values?
_car_size = [0.175, 0.1, 0.3, 0.05]  # [w h l r]
blind = 0  # For debugging w/o camera





def rs_init(width=640, height=480, fps=30):
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
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    try:
        # Start streaming
        pipeline.start(config)
    except Exception as err:
        print(err)

    return pipeline




def main():
    """
    Main Folkrace function
    :return: None
    """
    # initialize Realsense pipeline
    pl = rs_init()

    # Creat Car object and Path planner
    Car = Folkracer(car_size=_car_size, camera_car_offset=_camera_car_offset)
    pp = PathPlanner(pipeline=pl)

    # Start threads
    #Car.start()
    #pp.start()



    viewer(pl, Car, pp, not blind, ['pointcloud', 'frustum', 'axes', 'no-grid'])

    # Finish, close Car handler, Path planner and stream
    Car.end()
    pp.end()
    #pl.stop()
    # print(pp.planeFit())



if __name__ == '__main__':
    main()
