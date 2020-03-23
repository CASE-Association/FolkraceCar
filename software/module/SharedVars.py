"""
These are the share vars between parallel processes
"""

import multiprocessing as mp

Dist = mp.Value("d", 0)
Theta = mp.Value("d", 0)
Phi = mp.Value("d", 0)
Last_scan = mp.Value("d", 0.0)
SampleRate = mp.Value("d", 0.0)
