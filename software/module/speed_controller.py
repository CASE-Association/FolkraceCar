"""
Car speed controller
"""

from software.module.servo import *
import RPi.GPIO as GPIO



def main():
    motor = Servo(pin=33)
    motor.init()
    GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    try:
        print('\n\033[92m Servo test started\033[0m')
        while True:
            for ang in range(0, 60, 10):  # todo set proper limits
                motor.set_angle(ang)
                pin_state = GPIO.input(15)
                t_read = time.perf_counter()
                while GPIO.input(15) == pin_state:
                    if time.perf_counter() > t_read + 0.1:
                        break
                t_now = time.perf_counter()
                dt = t_now - t_read
                print(1/dt)
                #print(pin_state)


                time.sleep(3)

    except KeyboardInterrupt:
        motor.stop()
        GPIO.cleanup()
        print('\n\033[92m Servo test ended\033[0m')


if __name__ == '__main__':
    main()