#!/usr/bin/python3

import RPi.GPIO as GPIO
import multiprocessing as mp
from module import shared
import time


class FanController:
    def __init__(self, fan_pin, verbose=False, on_temp=60, off_temp=50):
        self._fan_pin = fan_pin
        self._t_on = on_temp
        self._t_off = off_temp
        self._verbose = verbose

        _p = mp.Process(target=self._run)
        #_p.daemon = True
        _p.start()

    def _run(self):
        dt = 5  # update delay
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self._fan_pin, GPIO.OUT)  # step
        fan_state = GPIO.HIGH
        while True:
            tFile = open('/sys/class/thermal/thermal_zone0/temp')
            temp = float(tFile.read())
            temp = temp / 1000

            if temp >= self._t_on:
                fan_state = GPIO.HIGH
            elif temp <= self._t_off:
                fan_state = GPIO.LOW
            GPIO.output(self._fan_pin, fan_state)
            if self._verbose:
                print("Cpu temp", round(temp, 2), "| fan is:", ('\033[94mOff\033[0m', '\033[92mOn\033[0m')[fan_state])
            shared.cpu_temp.value = temp
            time.sleep(dt)


def main():
    FanController(22, True)
    while True:
        time.sleep(0.01)


if __name__ == '__main__':
    main()

