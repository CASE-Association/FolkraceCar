"""
Setup servo actuation

if pwm is not working test following using root prev:

# Enable Pin 32 / PWM0
sudo busybox devmem 0x700031fc 32 0x45
sudo busybox devmem 0x6000d504 32 0x2

# Enable Pin 33 / PWM2
sudo busybox devmem 0x70003248 32 0x46
sudo busybox devmem 0x6000d100 32 0x00

"""

import Jetson.GPIO as GPIO
import time
import os

# os.system('sudo ./module/enable_pwm.sh')  # Enable PWM on pin 32 and 33  # fixme


class Servo:
    def __init__(self, pin, init_ang=0, pin_mode=GPIO.BOARD, no_checks=False):
        """
        Servo object
        :param pin: data pin to servo
        :param pin_mode: default GPIO.BOARD
        :param init_ang: initial angle
        :param no_checks: disable checks to increase performance
        """
        # Servo config
        self.angle_offset = 0  # todo
        self.angle_range = [-180, 180]  # in degrees

        # intrinsics
        self._no_checks = no_checks
        self._pin_mode = pin_mode
        self._ServoPWM = None  # GPIO.PWM object
        self._valid_pins = [32, 33]
        self._dc_range = [2.5, 12.5]  # todo tweak
        self._freq_range = [50, 200]  # todo check valid limits:
        self._pin = self._check_valid_pin(pin)
        self._angle = self._check_valid_ang(init_ang)  # angle in degrees
        self._frequency = 50  # [Hz]
        self._dutyCycle = self._ang_to_duty_cycle(self._angle)  # [%]

    @ property
    def Angle(self):
        """
        :return: angle in degrees
        """
        return self._angle

    @ property
    def Pin(self):
        return self._pin

    @ property
    def DutyCycle(self):
        return self._dutyCycle

    @property
    def Frequency(self):
        return self._frequency

    def set_angle(self, angle):
        """
        Set duty cycle corresponding to requested angle
        :param angle: requested angle
        :return: None
        """
        if self._check_is_initialized():
            self._angle = self._check_valid_ang(angle)
            self._set_duty_cycle(self._ang_to_duty_cycle(self._angle))

    def engage(self):
        """
        Engage servo output.
        Servo must first be initialized.
        :return: None
        """
        if self._check_is_initialized():
            self.set_angle(self._angle)

    def release(self):
        """
        Release servo output.
        Servo must first be initialized.
        :return: None
        """
        if self._check_is_initialized():
            self._set_duty_cycle(0)

    def init(self, frequency=50):  # todo auto init when object is created
        """
        Init Servo object
        :param frequency: optional initial frequency
        :return: None
        """

        self._frequency = self._check_valid_frequency(frequency)

        #GPIO.setwarnings(False)
        GPIO.setmode(self._pin_mode)
        GPIO.setup(self._pin, GPIO.OUT)

        self._ServoPWM = GPIO.PWM(self._pin, self._frequency)
        self._ServoPWM.start(0)

    def stop(self):
        if self._ServoPWM:
            self._ServoPWM.stop()
        else:
            print('Servo not initialized!')

    def _set_duty_cycle(self, duty_cycle):
        if self._check_is_initialized():
            self._dutyCycle = self._check_valid_duty_cycle(duty_cycle)
            self._ServoPWM.ChangeDutyCycle(self._dutyCycle)

    def _set_frequency(self, frequency):
        if self._check_is_initialized():
            self._ServoPWM.ChangeFrequency(self._check_valid_frequency(frequency))

    def _ang_to_duty_cycle(self, angle):
        """
        Recalcualte angle to required duty cycle.
        :param angle: requested angle
        :return: calculated duty cycle
        """

        self._angle = self._check_valid_ang(angle)

        #dc = round(2.5 + (ang + 90) / 18, 1)  # dutycycle range ~[2.5, 12.5]

        ang_lb = self.angle_range[0]
        ang_ub = self.angle_range[1]
        ang_delta = ang_ub - ang_lb
        dc_lb = self._dc_range[0]
        dc_ub = self._dc_range[1]
        dc_delta = dc_ub - dc_lb

        # normalize angle and apply to duty cycle range
        ang_norm = (self._angle - ang_lb) / ang_delta
        dc = dc_lb + dc_delta * ang_norm

        return dc

    def _check_valid_frequency(self, frequency):
        if self._no_checks:
            return frequency
        vrange = self._freq_range
        if frequency is None or not vrange[0] <= frequency <= vrange[1]:
            raise Exception('Invalid frequency!\n Valid range [{}]'.format(vrange))
        return frequency

    def _check_valid_duty_cycle(self, duty_cycle):
        vrange = self._dc_range
        bypass = self._no_checks or duty_cycle == 0

        if not bypass and (duty_cycle is None or not vrange[0] <= duty_cycle <= vrange[1]):
            raise Exception(('Invalid Duty Cycle!\n Valid range [{}]'.format(vrange)))
        return duty_cycle

    def _check_valid_ang(self, angle):
        if self._no_checks:
            return angle
        vrange = self.angle_range
        if angle is None or not vrange[0] <= angle <= vrange[1]:
            raise Exception(('Invalid angle!\nValid range [{}]'.format(vrange)))
        return angle

    def _check_is_initialized(self):
        """
        Check if servo GPIO is initialized.
        :return: True if initialized, else False
        """
        if self._no_checks or self._ServoPWM:
            return True
        else:
            raise Exception('Servo not initialized!')

    def _check_valid_pin(self, pin):
        if pin in self._valid_pins:
            return pin
        raise Exception(('Invalid pin!\nValid pins: {}'.format(self._valid_pins)))


def GPIO_cleanup():
    GPIO.cleanup()


def main():
    servo1 = Servo(pin=32)
    servo2 = Servo(pin=33)
    servo1.init()
    servo2.init()
    try:
        print('\n\033[92m Servo test started\033[0m')
        while True:
            for ang in range(-15, 15, 5):  # todo set proper limits
                servo1.set_angle(ang)
                servo2.set_angle(ang)
                print(servo1.Angle, servo1.Frequency, servo1.DutyCycle)
                time.sleep(1)

    except KeyboardInterrupt:
        servo1.stop()
        servo2.stop()
        GPIO.cleanup()
        print('\n\033[92m Servo test ended\033[0m')


if __name__ == '__main__':
    main()