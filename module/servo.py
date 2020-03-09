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

steer_servoPIN = 32
speed_servoPIN = 33
GPIO.setmode(GPIO.BOARD)
GPIO.setup(steer_servoPIN, GPIO.OUT)
GPIO.setup(speed_servoPIN, GPIO.OUT)

servo_steer = GPIO.PWM(steer_servoPIN, 50)  # GPIO for PWM with 50Hz
servo_speed = GPIO.PWM(speed_servoPIN, 50)  # GPIO for PWM with 50Hz
servo_steer.start(5)  # Initialization
servo_speed.start(5)  # Initialization
servo = [servo_speed, servo_steer]
try:
    print('Servo test started')
    p = servo[0]
    while True:
        for ang in range(-70, 120, 50):
            dc = round(2.5 + (ang + 90) / 18, 1)  # dutycycle range ~[2.5, 12.5]
            print(dc)
            p.ChangeDutyCycle(dc)
            time.sleep(0.5)
        """p = servo[0]
        p.ChangeDutyCycle(5)
        time.sleep(0.5)
        p.ChangeDutyCycle(7.5)
        time.sleep(0.5)
        p.ChangeDutyCycle(10)
        time.sleep(0.5)
        p.ChangeDutyCycle(12.5)
        time.sleep(0.5)
        p.ChangeDutyCycle(10)
        time.sleep(0.5)
        p.ChangeDutyCycle(7.5)
        time.sleep(0.5)
        p.ChangeDutyCycle(5)
        time.sleep(0.5)
        p.ChangeDutyCycle(2.5)
        time.sleep(0.5)"""
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
    print('\nServo test ended')
