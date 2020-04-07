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
    def __init__(self, tacho_pin, sample_interval=0.05, kp=500, ki=1000, verbose=False):

        # extrinsics
        self.max_speed = 10  # [m/s]
        self.verbose = verbose
        self.process = None
        self.wheel_diameter = 0.5  # [m]

        # intrinsics
        self._speed = mp.Value('d', 0.0)  # Calculated speed, init speed 0.0 [m/s]
        self._target_speed = mp.Value('d', 0.0)  # the speed that is requested  # [m/s]
        self._power = mp.Value('i', 0)  # the controller power variable, span [-100, 100]%
        self._pin = tacho_pin
        self._wheel_circumference = self.wheel_diameter * np.pi  # [m]
        self._motor_wheel_rpm_ratio = 3  # the ratio between motor axle speed and wheel rpm [Estimation]
        self._run = mp.Value('i', 1)

        # Speed variables
        self._pulse = 0
        self._ppr = 17  # Pulse/ wheel revolution  (est 1pulse/11mm)
        self._rpm = 0
        self._t_last_count = 0  # time of last count

        # PI control values  # todo implement MPC!
        self._kp = kp
        self._ki = ki
        self._sample_interval = sample_interval  # sample time in seconds
        self._windup_guard_val = 150  # Maximum Integration

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
        self.process = mp.Process(target=self.run, args=(self._speed, self._target_speed, self._power,))
        # Start speed controller
        self.process.start()

    def end(self):
        if self.process is not None:
            self._run.value = 0
            time.sleep(0.5)
            if self.process.is_alive():
                self.process.terminate()
                if self.verbose:
                    print('SpeedController was terminated')
            elif self.verbose:
                print('SpeedController have ended with exitcode({})'.format(self.process.exitcode))

    def set_speed(self, speed):
        self._target_speed.value = np.clip(speed, -self.max_speed, self.max_speed)

    def _get_speed(self):
        """
        Get how many pulses since last count and divide by time-delta to get w/s.
        Multiply w/s with wheel circumference to the speed in m/s.
        :return: car speed in m/s
        """
        t_now = time.perf_counter()
        delta_t = t_now - self._t_last_count
        p = self._pulse
        if p:  # if pulses > 0, reset counter and timer
            self._t_last_count = t_now
            self._pulse = 0
        else:
            return 0.0
        rps = (p / self._ppr) * delta_t  # wheel rotations per second.
        mps = rps * self._wheel_circumference
        return mps

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
                # t = time.strftime("%H:%M:%S", time.gmtime())
                # print('[{}]: Speed Controller in safe mode!\n'
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
        # print(channel, self._pulse)

    def run(self, speed, target_speed, power, alpha=0.9):
        """
        Speed controller loop function
        :param speed: Output current speed [multiprocessing.Value]
        :param target_speed: Input controller target speed [multiprocessing.Value]
        :param power: Output controller power
        :param alpha: Smoothing factor of speed reading
        :return: None
        """

        # Setup tachometer pulse callback function
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # todo remove when hw pull-up implemented
        GPIO.add_event_detect(self._pin, GPIO.RISING, callback=self._cb_new_pulse)

        # init PI values
        t_last = time.perf_counter()
        self._t_last_count = t_last
        I = 0
        s = 0
        while self._run:
            # PI control
            new_s = self._get_speed()  # Get car speed
            s = alpha * new_s + (1 - alpha) * s  # Smooth speed reading
            speed.value = s
            err = target_speed.value - s
            t_now = time.perf_counter()
            delta_t = t_now - t_last
            # calculate integral part
            I += err * delta_t
            I = np.clip(I, -self._windup_guard_val, self._windup_guard_val)  # prevent integration windup

            #print('E: {}, P: {}, I: {}'.format(round(err, 2),  int(self._kp * err), int(self._ki * I)))

            # Calculate power output and constrain to [-100, 100]%
            pwr = np.clip(int(self._kp * err + self._ki * I), -100, 100)
            pwr = self._fault_guard(pwr, timeout=1, safemode_power=50)  # Check if safe to give power.
            power.value = pwr
            t_last = t_now

            # sleep for reminder of sampling time.
            t_sleep = float(np.clip((2 * self._sample_interval - delta_t), 0, self._sample_interval))
            time.sleep(t_sleep)


def main():
    sc = SpeedControl(tacho_pin=36, verbose=True)
    sc.set_speed(0.03)
    sc.start()
    # init esc
    speed_q = mp.Queue()
    motor = Servo(speed_q, 32)
    motor.start()
    speed_q.put(-1)  # esc unlocking
    time.sleep(0.1)

    t_start = time.perf_counter()

    test_duration = 5  # [s]

    dt = 0.01

    while time.perf_counter() < t_start + test_duration:
        try:
            print('Time: {:2.0f}s,  Speed: {:4.3f}m/s, Power: {}%'
                  .format(round(time.perf_counter(), 0), sc.speed, sc.power))
            time.sleep(dt)
            pwr = np.clip(sc.power, -100, 100)
            speed_q.put(-pwr)  # apply power [inverted]

        except KeyboardInterrupt:
            break
    sc.end()
    motor.end()


if __name__ == '__main__':
    main()
