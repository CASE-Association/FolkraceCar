#!/usr/bin/python3
import multiprocessing as mp
import time
import RPi.GPIO as GPIO
import numpy as np
from scipy.integrate import cumtrapz


#  todo convert rpm control to speed control

class SpeedControl:
    def __init__(self, tach_pin, kp=0.1, ki=0.5, motor_wheel_rpm_ratio=1):
        # intrinsics
        self._speed = mp.Value('d', 0.0)  # Calculated speed, init speed 0.0
        self._target_speed = mp.Value('d', 0.0)  # the speed that is requested
        self._power = mp.Value('i', 0)  # the controller power variable, span [-100, 100]% where -100 is full power in reverse direction.
        self._pin = tach_pin
        self._motor_wheel_rpm_ratio = motor_wheel_rpm_ratio  # the ratio between motor axle speed and wheel rpm
        self._run = mp.Value('i', 1)
        self.p = None
        # PI control values  # todo implement MPC!
        self._kp = kp
        self._ki = ki
        self._sample_interval = 0.01
        self._windup_guard_val = 100  # Maximum Integral actuation

        # extrinsics
        self.max_rpm = 1000

    @property
    def speed(self):
        """
        The speed retrieved from speed sensors
        :return: curent cehicle speed
        """
        return self._speed.value

    @property
    def target_speed(self):
        """
        Set target speed by using the Set_speed funciton
        :return: target speed
        """
        return self._target_speed.value

    @property
    def power(self):
        """
        :return: The controlled power
        """
        return self._power.value

    def start(self):
        self.p = mp.Process(target=self.run, args=(self._speed, self._target_speed, self._power,))
        self.p.start()

    def end(self):
        if self.p is not None:
            self._run.value = 0
            time.sleep(0.5)
            if self.p.is_alive():
                self.p.terminate()
                print('SpeedController was terminated')
            else:
                print('SpeedController have ended with exitcode({})'.format(self.p.exitcode))

    def set_speed(self, speed):
        self._target_speed.value = np.clip(speed, -self.max_rpm, self.max_rpm)

    def get_rpm(self):  # todo look into using callback
        t_start = time.perf_counter()
        state = GPIO.input(self._pin)
        while GPIO.input(self._pin) == state:  # wait for input to change
            if time.perf_counter() - t_start > 0.1:  # took more than 100ms to change state, return rmp=0
                return 0.0
        t_end = time.perf_counter()
        dt = t_end - t_start
        rpm = 1 / dt
        return rpm

    def run(self, speed, target_speed, power):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        t_last = time.perf_counter()
        I = 0
        while self._run:
            # PI control
            s = self.get_rpm()
            speed.value = s
            t = target_speed.value
            err = t - s
            t_now = time.perf_counter()
            delta_t = t_now - t_last
            if delta_t > self._sample_interval:
                I += err * delta_t
                I = np.clip(I, -self._windup_guard_val, self._windup_guard_val)  # prevent integrating windup
                t_last = t_now
                power.value = np.clip(int(self._kp * err + self._ki * I), -100, 100)







def main():
    sc = SpeedControl(36)
    sc.set_speed(100)
    sc.start()

    while True:
        try:
            print('Speed: {}, Power: {}'.format(round(sc.speed, 2), sc.power))
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    sc.end()


if __name__ == '__main__':
    main()
