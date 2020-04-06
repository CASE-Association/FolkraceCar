#!/usr/bin/python3
import multiprocessing as mp
import time
import RPi.GPIO as GPIO
import numpy as np
from software.module.servo import Servo

#  todo
#   convert rpm control to speed control
#   convert speed output to control pwm_pin instead


class SpeedControl:
    def __init__(self, tacho_pin, sample_interval=0.01, kp=0.005, ki=0.05, motor_wheel_rpm_ratio=0.5, verbose=False):
        # intrinsics
        self._speed = mp.Value('d', 0.0)  # Calculated speed, init speed 0.0
        self._target_speed = mp.Value('d', 0.0)  # the speed that is requested  # todo change from rpm to m/s
        self._power = mp.Value('i',
                               0)  # the controller power variable, span [-100, 100]% where -100 is full power in reverse direction.
        self._pin = tacho_pin
        self._motor_wheel_rpm_ratio = motor_wheel_rpm_ratio  # the ratio between motor axle speed and wheel rpm
        self._run = mp.Value('i', 1)

        # Speed variables
        self._pulse = 0
        self._ppr = 0.2  # Pulse/revolution
        self._rpm = 0
        self._t_last_count = 0  # time of last count

        # PI control values  # todo implement MPC!put
        self._kp = kp
        self._ki = ki
        self._sample_interval = sample_interval  # sample time in seconds
        self._windup_guard_val = 100  # Maximum Integral actuation

        # extrinsics
        self.max_rpm = 10000
        self.verbose = verbose
        self.p = None

    @property
    def speed(self):
        """
        The speed retrieved from speed sensors
        :return: current vehicle speed
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
        # Create speed controller Process
        self.p = mp.Process(target=self.run, args=(self._speed, self._target_speed, self._power,))
        # Start speed controller
        self.p.start()

    def end(self):
        if self.p is not None:
            self._run.value = 0
            time.sleep(0.5)
            if self.p.is_alive():
                self.p.terminate()
                if self.verbose:
                    print('SpeedController was terminated')
            elif self.verbose:
                print('SpeedController have ended with exitcode({})'.format(self.p.exitcode))

    def set_speed(self, speed):
        self._target_speed.value = np.clip(speed, -self.max_rpm, self.max_rpm)

    def get_rpm_old(self):  # todo look into using callback
        t_start = time.perf_counter()
        state = GPIO.input(self._pin)
        while GPIO.input(self._pin) == state:  # wait for input to change
            if time.perf_counter() - t_start > 0.1:  # took more than 100ms to change state, return rmp=0
                return 0.0
        t_end = time.perf_counter()
        dt = t_end - t_start
        rpm = 1 / dt
        return rpm

    def get_rpm(self):
        """
        Get how many pulses since last count and divide by time-delta to get rpm.
        :return: Motor RPM
        """
        t_now = time.perf_counter()
        delta_t = t_now - self._t_last_count
        p = self._pulse
        if p:  # if pulses > 0, reset counter and timer
            self._t_last_count = t_now
            self._pulse = 0
        else:
            return 0.0
        rpm = p * self._ppr * (60 / delta_t)  # multiply by 60 and ppr to get RPM
        return rpm

    def _fault_guard(self, power, timeout=1, safemode_power=10):
        """
        Check if control action have effect. If not, constrain power
        :param power: Requested power
        :param timeout: Constrain if no action before timeout
        :param safemode_power: Max safe power if timed out.
        :return: Safeguarded power.
        """
        if power != 0 and self._rpm == 0 and time.perf_counter() - self._t_last_count > timeout:
            if self.verbose and power != safemode_power:
                pass
                #t = time.strftime("%H:%M:%S", time.gmtime())
                #print('[{}]: Speed Controller in safe mode!\n'
                #      'Power applied but no movement detected!\n'
                #      'Maximum power:{}%'.format(t, safemode_power))
            return np.clip(power, -safemode_power, safemode_power)
        return power

    def _cb_new_pulse(self, channel=None):
        """
        Hall pin transition callback function. Increment pulse each pin transition.
        :return: None
        """
        self._pulse += 1
        #print(channel, self._pulse)

    def run(self, speed, target_speed, power):
        """
        Speed controller loop function
        :param speed: Output current speed [multiprocessing.Value]
        :param target_speed: Input controller target speed [multiprocessing.Value]
        :param power: Output controller power
        :return: None
        """

        # Setup tachometer pulse callback function
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # todo remove when hw pull-up implemented
        GPIO.add_event_detect(self._pin, GPIO.BOTH, callback=self._cb_new_pulse)

        t_last = time.perf_counter()
        I = 0
        while self._run:

            # PI control
            s = self.get_rpm()  # Get angle velocity
            speed.value = s
            err = target_speed.value - s
            t_now = time.perf_counter()
            delta_t = t_now - t_last
            # calculate integral part
            I += err * delta_t
            I = np.clip(I, -self._windup_guard_val, self._windup_guard_val)  # prevent integration windup
            t_last = t_now

            # Calculate power output and constrain to [-100, 100]%
            pwr = np.clip(int(self._kp * err + self._ki * I), -100, 100)
            power.value = self._fault_guard(pwr, timeout=1, safemode_power=10)  # Check if safe to give power.

            time.sleep(self._sample_interval)


def main():
    sc = SpeedControl(tacho_pin=36, verbose=True)
    sc.set_speed(100)
    sc.start()
    speed_q = mp.Queue()
    motor = Servo(speed_q, 32)
    motor.start()

    speed_q.put(-10)  # esc unlocking
    time.sleep(0.1)

    while True:
        try:
            print('Time: {}  Speed: {}, Power: {}'.format(round(time.perf_counter(), 0), round(sc.speed, 2), sc.power))
            time.sleep(0.05)
            speed_q.put(np.clip(sc.speed, 0, 30))
        except KeyboardInterrupt:
            break
    sc.end()
    motor.end()

if __name__ == '__main__':
    main()
