#!/usr/bin/python3

import time
from multiprocessing import Process, Queue
import RPi.GPIO as GPIO


class Servo:
    def __init__(self, queue, pin, verbose=False):
        self.q = queue
        self.servopin = pin
        self.verbose = verbose
        # intrinsic const, must be set before start!
        self.ub = 90  # angle upper bound
        self.lb = -90  # anfle lower bound
        self.ang_offset = 0
        self.dc = 0

    def start(self):
        self.p = Process(target=self.run, args=((self.q),))
        #self.q.daemon = True
        self.p.start()

    def end(self):
        self.q.put('END')
        self.p.join(timeout=1)

    def map(self, value):
        """
        Servo running at 50 Hz -> 20 ms per cycle,
        control: high for 1ms to 2ms controls angle
        dutycycle value is from 0 to 100,
        value from controller is -1 to 1
        """
        # at 0 we want to pulse to be 1.5 Ms -> 0.075 duty cycle

        value = ((value + 1) / 2)  # now 0-1
        value = value * (0.1 - 0.05) + 0.05
        # map to 0-100
        return value * 100

    def run(self, queue):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servopin, GPIO.OUT)  # step
        self.pwm = GPIO.PWM(self.servopin, 50)

        init_dc = self.map(self.ang_offset/180)
        self.pwm.start(init_dc)  # center output

        while True:
            try:
                #inp = queue.get_nowait()  # get queue input
                inp = queue.get()  # get queue input
                if inp == 'END':
                    queue.close()
                    print('\n\033[92m Servo process ended\033[0m')
                    break
                elif -90 <= inp <= 90:
                    self.dc = self.map(inp/180 + self.ang_offset/180)  # normalize angle and map to dutycycle
                    self.pwm.ChangeDutyCycle(self.dc)  # todo contol values
                    if self.verbose:
                        print("[ServoPin", self.servopin, "] Dutycycle changed to: ",
                              round(inp, 2), "deg, dc= ", round(self.dc, 2))
                else:
                    if inp.any() and self.verbose:
                        print('invalid data:', inp)
            except:
                time.sleep(0.001)
                pass


if __name__ == "__main__":

    servo = Servo(Queue(), pin=32, verbose=True)
    servo.start()
    i = 1
    try:
        while True:
            time.sleep(0.5)

            servo.q.put(i*80 + 10)

            i = -i



    except KeyboardInterrupt:
        pass

    servo.q.put('END')
