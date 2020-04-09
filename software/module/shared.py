import multiprocessing as mp
import numpy as np
import ctypes as c
from module.config import *

"""
Global variables shared between processes
"""

# Raw Image array
if IMAGERAW:
    raw_image = mp.RawArray('d', IMG_SIZE[0] * IMG_SIZE[1] * IMG_SIZE[2])
else:
    raw_image = None

# Raw Verts array
if VERTSRAW:
    raw_verts = None

# Speed
# todo

# Heading
# todo

# etc...


def nparray_to_rawarray(arr):
    raw_arr = mp.RawArray(c.c_double, int(np.prod(arr.shape)))
    np.frombuffer(raw_arr).reshape(arr.shape)[...] = arr
    return raw_arr

def rawarray_to_nparray(raw_arr, shape):
    return np.frombuffer(raw_arr).reshape(shape)
