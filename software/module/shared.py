import multiprocessing as mp
import logging
import numpy as np
import ctypes as c
from module.config import *

"""
Global variables shared between processes
"""
global raw_image

# Loglevel
LOG_LEVEL = logging.ERROR

# Raw Image array
t_raw_image = mp.Value('f', 0.0)  # timestamp of last frame

if IMAGERAW:
    raw_image = mp.RawArray('d', IMG_SIZE[0] * IMG_SIZE[1] * IMG_SIZE[2])

# Raw Verts array
if VERTSRAW:
    raw_verts = None

# Speed
speed = mp.Value('f', 0.0)

# Heading
steer = mp.Value('f', 0.0)

# Cpu temp
cpu_temp = mp.Value('f', 0.0)

# Battery current
I_b = mp.Value('f', 0.0)

# Battery voltage
U_b = mp.Value('f', 0.0)

# Motor current
I_m = mp.Value('f', 0.0)

#

def nparray_to_rawarray(arr):
    global raw_image
    #raw_arr = mp.RawArray(c.c_double, int(np.prod(arr.shape)))
    np.frombuffer(raw_image).reshape(arr.shape)[...] = arr
    return raw_image

def rawarray_to_nparray(raw_arr, shape):
    return np.frombuffer(raw_arr).reshape(shape)
