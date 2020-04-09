
"""
Configuration module
"""

# todo change to sys/argument varables

WEBGUI = True
STREAM = True
IMAGERAW = WEBGUI  # generate raw image if Web GUI is used
VERTSRAW = False


frame_width = 424
frame_height = 240
fps = (0, 60)[IMAGERAW]  # disable color if not needed
dfps = 90
FOV = 50  # [deg]
camera_car_offset = [0.01, 0,
                     -2.05]  # The offset from camera to car pivot point. [w h l] [m]
#car_size = [0.2, 0.1, 0.3, 0.05]  # [w h l r]
car_size = [0.2, 0.05, 0.12, 0.05]  # [w h l r]


# I/O CONFIG

FAN = 22
STEER_SERVO = 32
MOTOR_PWM = 33
HALL_SENSOR = 36


# Image config
IMG_SIZE = (240, 424, 3)

DECIMATION = 3

# PathFinder
PATH_SMOOTHING = 0.25