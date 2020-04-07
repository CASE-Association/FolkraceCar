"""
## This is Code developed by the CASE Association, built on code and examples
## from Intel Corporation.
## License: MIT See LICENSE file in root directory.
## Modifications Copyright (c) 2020 CASE (Chalmers Autonomous Systems and Electronics)
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

TODO
    Gwt ADC to work
    Get rpm input to work
    hw test
"""
from module.carhandler import *
from module.pathplanner import *
from module.servo import *
from module.config import *
from module.fancontroller import *
import time
import os


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
    FanController(fan_pin=22)

    #   Car is the process handling the dynamics of the car
    car = CarHandler(car_size=car_size, camera_car_offset=camera_car_offset)

    #   Actuator initialization  # todo move to car_handler
    steer_servo = Servo(pin=32, queue=Queue(), verbose=True)
    speed_servo = Servo(pin=33, queue=Queue(), verbose=True)
    steer_servo.ang_offset, steer_servo.ub, steer_servo.lb = 0, 45, -45
    speed_servo.ang_offset, steer_servo.ub, steer_servo.lb = 0, 45, -45
    steer_servo.start()
    speed_servo.start()

    q_path = mp.Queue()
    PP = mp.Process(target=path_plan, args=(car, q_path), name='path_planners')
    # Start threads
    # Car.start()
    # run the path planner as daemon so it terminates with the main process
    PP.daemon = True
    PP.start()
    time.sleep(1)  # give process time to start

    #  Test code

    _fps = 30
    _phi = 0.0
    opt_theta = 0.0
    opt_phi = 0.0
    dist = 0
    _t_last_reading = 0.0
    _t_last_update = 0.0
    _max_speed = 180
    try:
        while True:
            q_data = q_path.get()
            _max_dist, theta = q_data.get('dist', -1), q_data.get('theta', 0.0)

            _t_new_reading = q_data.get('last_reading', 0)
            hz = 1 / (_t_new_reading - _t_last_reading)
            _t_last_reading = _t_new_reading
            _fps = (0.9 * _fps + 0.1 * hz)
            dist = (0.9 * dist + 0.1 * _max_dist)
            opt_theta = round(0.99 * opt_theta + 0.01 * theta, 1)

            if _t_last_update + 0.05 < time.perf_counter():
                os.system('clear')
                print('\n\033[92m                 CASE FolkRacer\033[0m')
                print('Dist {:4.2f}m | Theta  {:5.1f}deg | Phi {:5.1f}|  f: {:5.2f}Hz'
                      .format(round(dist, 2), opt_theta, opt_phi, round(_fps, 2)))
                _turn = min(max(round(theta), -25), 25)
                print('|' * (25 + _turn), '\033[92mA\033[0m', '|' * (25 - _turn))
                print('Speed: 00.0')
                print('\nCtrl-C to end')

                speed = max(min(_max_speed, (3 * dist - 1) ** 2), -_max_speed)  # crude speed setup
                steer = theta * 4
                # update steer and speed value
                steer_servo.q.put(steer)
                speed_servo.q.put(speed)
                _t_last_update = time.perf_counter()

    except KeyboardInterrupt:
        pass

    # Cleanup

    car.end()  # end car thread
    q_path.put("END")  # send END msg to shutdown child processes
    speed_servo.end()
    steer_servo.end()
    #PP.join()  # todo fix process staying alive

    print('\n\033[92m Main process ended\033[0m')


if __name__ == '__main__':
    main()
