"""
## This is Code developed by the CASE Association, built on code and examples
## from Intel Corporation.
## License: MIT See LICENSE file in root directory.
## Modifications Copyright (c) 2020 CASE (Chalmers Autonomous Systems and Electronics)
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

TODO
    Get ADC to work
    Get Webcontrol to work
    Get OLED screen to work
"""
from module.carhandler import *
from module.pathplanner import *
from module.servo import *
from module.config import *
from module.fancontroller import *
from module.speedcontroller import *
import module.shared as shared
from threading import Thread
import time
import os

from module.WebControl import web


def plot_pointcloud(points, only_closest=False, step=3):
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

    if only_closest:
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
    # init

    # Create a fan controller
    FanController(fan_pin=FAN, on_temp=55, off_temp=45)  # todo activate when needed

    # Create Speed controller  # todo move to CarHandler
    sc = SpeedControl(tacho_pin=HALL_SENSOR)
    # sc.start()  # todo activate when needed

    #   CarHandler is the process handling the dynamics of the car
    car = CarHandler(car_size=car_size, camera_car_offset=camera_car_offset)
    # car.start() # todo activate when needed

    #   Actuator initialization  # todo move to CarHandler
    steer_servo = Servo(pin=STEER_SERVO, queue=Queue(), verbose=False, name='Steer_servo')
    speed_servo = Servo(pin=MOTOR_PWM, queue=Queue(), verbose=False, name='Motor_servo')
    steer_servo.start()
    speed_servo.start()

    # PathFinder initialization and start
    pf = PathFinder(car)
    pf.start()

    # start web server
    if WEBGUI:
        wp = Process(target=web.main, name='WebController')
        wp.start()

    _fps = 30
    _phi = 0.0
    _t_last_reading = _t_last_update = time.perf_counter()
    _max_speed = 90
    lag = 0
    str_print = ""
    try:
        while True:
            # Get data from PathFinder. Block until new data to prevent unnecessary calculations
            q_data = pf.q.get()

            # get timestamp of last reading
            _t_new_reading = q_data.get('last_reading', 0)
            dt = _t_new_reading - _t_last_reading
            if dt:
                hz = 1 / dt
            else:
                continue  # no new data
            _lag = (time.perf_counter() - _t_new_reading) * 1000  # reading delay in ms.
            lag = 0.995 * lag + 0.005 * _lag
            _t_last_reading = _t_new_reading
            _fps = 0.999 * _fps + 0.001 * hz

            # Get path data
            if conf.FOLKRACE:
                dist, theta = q_data.get('dist', -1), q_data.get('theta', 0.0)
            else:
                dist, theta = shared.speed.value * 50, shared.steer.value * 25

            opt_theta = round(theta, 1)
            _turn = min(max(round(theta), -25), 25)

            if _t_last_update + 0.01 < time.perf_counter():
                _str_1 = '\n\033[92m                    CASE FolkRacer\033[0m\n ' \
                         'Dist {:4.2f}m | Theta {:5.1f}deg | Lag {:4.1f}ms | f: {:3.0f}Hz' \
                    .format(round(dist, 2), opt_theta, lag, round(_fps, 2))

                _str_2 = '\n\033[92m' + '-' * 53 + '\033[0m\n' + \
                         ('|' * (25 + _turn)) + \
                         '\033[92m' + (' ↑ ', ' ↓ ')[sc.speed < 0] + \
                         '\033[0m' + ('|' * (25 - _turn)) + \
                         '\n\033[92m' + '-' * 53 + '\033[0m\n'

                _str_3 = ' Steer: {:2.0f}deg\n' \
                         ' Speed: {:2.3f}m/s\n' \
                         ' Power: {}%\n' \
                         ' Cpu Temp: {:2.1f}c\n\n' \
                         ' Ctrl-C to end' \
                    .format(theta * 4, sc.speed, sc.power, shared.cpu_temp.value)

                _str = _str_1 + '\n' + _str_2 + '\n' + _str_3
                if _str != str_print:
                    os.system('clear')
                    str_print = _str
                    print(str_print)
                else:
                    pass
                    #print('.', end='')

                speed = max(min(_max_speed, (3 * dist - 1) ** 2), -_max_speed)  # crude speed setup
                steer = theta * 4
                # update steer and speed value
                sc.set_speed(speed)

                steer_servo.q.put(np.clip(steer, -90, 90))
                speed_servo.q.put(np.clip(sc.power, -50, 50))
                _t_last_update = time.perf_counter()


    except KeyboardInterrupt:
        pass

    # Cleanup
    pf.end()
    car.end()
    speed_servo.end()
    steer_servo.end()

    print('\n\033[92m Main process ended\033[0m')


if __name__ == '__main__':
    main()
