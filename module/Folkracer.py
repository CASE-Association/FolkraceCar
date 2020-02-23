"""
Folkrace Car code

Developed by:
Stefan Larsson of CASE Assosiation 2020
"""

import numpy as np
import threading as mt
import time


class Folkracer:
    def __init__(self, car_size, camera_car_offset):
        """
        Main Car object.
        Defines the model of car and camera position relation.
        Holds current value of speed and steering angle.
        These can be passed from e.g. a Path planner to control the Car.
        :param car_size: dimension of car in pointcloud dimension
        :param camera_car_offset: offsent from car origin to camera origin
        """
        #  Define car size [width, height, length, wheel radius]
        if not car_size:
            self.size = [0.2, 0.1, 0.3, 0.05]
        else:
            self.size = car_size
        if not camera_car_offset:
            self.camera_offset = np.zeros((3, 1))
        else:
            self.camera_offset = camera_car_offset
        self.steer = 0
        self.speed = 0
        self.brake = True
        self.running = False
        self.state = [self.steer, self.speed, self.brake]

        # todo this will be the driving vector of which the car will be regulated to follow
        #  returns longest possible driving direction.
        #  OBS endpoint may have contact with object
        self.drive_vector = np.zeros((1, 2))   # [x, y]

    def start(self):
        self.running = True
        car_hand = mt.Thread(target=self.car_handler, name='car_handler')
        try:
            car_hand.start()
        except Exception as err:
            print("Error: unable to start car_handler thread, err: {}".format(err))
        return car_hand

    def end(self):
        self.running = False

    def car_handler(self):
        while self.running:
            print('Steer: {} Speed: {} Is running: {}'.format(self.steer, self.speed, self.running))
            time.sleep(2)
        print('car_handler has ended')

