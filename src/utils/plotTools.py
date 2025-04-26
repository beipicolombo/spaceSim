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
       
         
# 3d plot
def plotOrbit3d(tsPos_I, planetRad = 1):
    fig = plt.figure()
    
    ax = fig.add_subplot(111, projection='3d')

    # Plot planet surface
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(planetRad*x, planetRad*y, planetRad*z,  rstride = 4, cstride = 4, color = 'b', linewidth = 0, alpha = 0.2)
    
    # Plot planet inertial frame origin
    dotRatio = 5/100
    ax.plot_surface(planetRad*dotRatio*x, planetRad*dotRatio*y, planetRad*dotRatio*z,  rstride = 4, cstride = 4, color = 'r', linewidth = 0, alpha = 1.0) 

    # Plot planet equator
    horiz_front = np.linspace(0, 2*np.pi, 100)
    ax.plot(planetRad*np.sin(horiz_front), planetRad*np.cos(horiz_front), 0, color = 'k', linestyle = 'dashed')

    # Plot vernal equinox axis
    ax.quiver(0, 0, 0,1.5*planetRad, 0, 0, color = 'r',  arrow_length_ratio = 0.05)

    # Plot starting position
    dotRatio = 5/100
    ax.plot_surface(tsPos_I.dataVec[0,0]+planetRad*dotRatio*x, tsPos_I.dataVec[0,1]+planetRad*dotRatio*y, tsPos_I.dataVec[0,2]+planetRad*dotRatio*z,  rstride = 4, cstride = 4, color = 'b', linewidth = 0, alpha = 1.0) 

    # Plot starting position
    dotRatio = 5/100
    ax.plot_surface(tsPos_I.dataVec[-1,0]+planetRad*dotRatio*x, tsPos_I.dataVec[-1,1]+planetRad*dotRatio*y, tsPos_I.dataVec[-1,2]+planetRad*dotRatio*z,  rstride = 4, cstride = 4, color = 'r', linewidth = 0, alpha = 1.0) 


    # Plot orbit
    ax.plot3D(tsPos_I.dataVec[:,0], tsPos_I.dataVec[:,1], tsPos_I.dataVec[:,2])

    # TBW: plot lines of interest
    # ===============================
    #calculate vectors for "vertical" circle
    # elev = 10.0
    # rot = 80.0 / 180 * np.pi
    # a = np.array([-np.sin(elev / 180 * np.pi), 0, np.cos(elev / 180 * np.pi)])
    # b = np.array([0, 1, 0])
    # b = b * np.cos(rot) + np.cross(a, b) * np.sin(rot) + a * np.dot(a, b) * (1 - np.cos(rot))
    # ax.plot(np.sin(u),np.cos(u),0,color='k', linestyle = 'dashed')
    # vert_front = np.linspace(np.pi / 2, 3 * np.pi / 2, 100)
    # ax.plot(a[0] * np.sin(u) + b[0] * np.cos(u), b[1] * np.cos(u), a[2] * np.sin(u) + b[2] * np.cos(u),color='k', linestyle = 'dashed')
    # ax.plot(a[0] * np.sin(vert_front) + b[0] * np.cos(vert_front), b[1] * np.cos(vert_front), a[2] * np.sin(vert_front) + b[2] * np.cos(vert_front),color='k')

    # Set aspects
    ax.set_aspect('equal')
    ax.view_init(elev = 20.0, azim = -20.0)

    # Add plot informations
    ax.set_xlabel("x " + tsPos_I.unit)            
    ax.set_ylabel("y " + tsPos_I.unit)
    ax.set_zlabel("z " + tsPos_I.unit)
    plt.title(tsPos_I.name)

    plt.show()