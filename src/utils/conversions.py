# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import matplotlib.pyplot as plt
import ephem


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180
       
         
# --------------------------------------------------
# Conversions
# --------------------------------------------------
def latLonToCartesian(lat, lon, rad):
    x = rad * np.cos(lat) * np.cos(lon)
    y = rad * np.cos(lat) * np.sin(lon)
    z = rad * np.sin(lat)

    coord = np.array([x, y, z])

    return coord