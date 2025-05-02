# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import src.utils.constants as const
import numpy as np
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
# import cartopy.crs as ccrs
# import cartopy
import time as ti


# Retrieve useful constants
pi  = np.pi
deg2rad = pi/180 # TBW: use constants module


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class OrbitInitParam():
    def __init__(self):
        self.perAltitudeInit = 500 * 1e3 # [m]
        self.ecc = 0.01
        self.inc = 10 * deg2rad # [rad]
        self.raan = 10 * deg2rad # [rad]
        self.argPer = 90 * deg2rad # [rad]
        self.ta = 0 * deg2rad # [rad]


class OrbitContext():
    def __init__(self):
        self.longPer = 0
        self.perRad = 0
        self.apoRad = 0
        self.angMom_I = np.array([0, 0, 0])
        self.orbitPulse = 0
        self.orbitPeriod = 0

    def set(self, keplerElem, mu):
        self.orbitPulse = np.sqrt(mu / pow(keplerElem.sma, 3))
        self.orbitPeriod = 2*pi / self.orbitPulse 


class Kepler:
    def __init__(self):
        self.sma = 0
        self.ecc = 0
        self.inc = 0
        self.raan = 0
        self.argPer = 0
        self.ta = 0
        self.longPer = 0
        self.perRad = 0
        self.apoRad = 0
        self.angMom_I = np.array([0, 0, 0])

    def set(self, pos_I, vel_I, mu):
        (sma ,ecc, inc, raan, argPer, taEp, lonPer, lonEp, perRad, apoRad, angMomVec) = posVelInertialToElements(pos_I, vel_I, mu)
        self.sma = sma
        self.ecc = ecc
        self.inc = inc
        self.raan = raan
        self.argPer = argPer
        self.ta = taEp
        self.longPer = lonPer
        self.perRad = perRad
        self.apoRad = apoRad
        self.angMom_I = angMomVec

class Orbit:
    def __init__(self, mu, pos_I = np.array([6870.993129, 0, 0]), vel_I = np.array([0, 7500.85563415, 1322.6032267])):
        self.pos_I = pos_I
        self.vel_I = vel_I
        self.keplerElem = Kepler()
        self.orbitContext = OrbitContext()

        self.keplerElem.set(pos_I, vel_I, mu)
        self.orbitContext.set(self.keplerElem, mu)

    def posVelToVec(self):
        vec = np.concatenate((np.array(self.pos_I), np.array(self.vel_I)),0)
        return vec
        
    def propagate(self, simParam, mu):
        Ts = simParam.Ts
        
        # Propagate orbit
        xPrev = self.posVelToVec()
        k1 = dotPosVel(xPrev, 0, mu)
        x2 = xPrev + k1/2
        k2 = dotPosVel(x2, 0, mu)
        x3 = xPrev + k2/2
        k3 = dotPosVel(x3 , 0, mu)
        x4 = xPrev + k3
        k4 = dotPosVel(x4, 0, mu)
        xNext = xPrev + Ts/6 * (k1 + 2*k2 + 2*k3 + k4)
        
        (posNext_I, velNext_I) = vecToPosVel(xNext)
        
        # Update position / velocity state
        self.pos_I = posNext_I
        self.vel_I = velNext_I
        # Update kepler elements state
        self.keplerElem.set(self.pos_I, self.vel_I, mu)
        self.orbitContext.set(self.keplerElem, mu)

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def propagateOrbit(simParam, modelsBus):
    # Initialize ouput bus
    modelsBusOut = modelsBus
    pos_I = modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].value
    vel_I = modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["vel_I"].value
    mu = const.earthMu # TBW => use single common parameter for gravity model config
    # Get orbit object
    orbitObj = Orbit(mu, pos_I = pos_I, vel_I = vel_I)
    # Propagate
    orbitObj.propagate(simParam, mu)
    # Update output bus signals
    modelsBusOut.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].update(orbitObj.pos_I)
    modelsBusOut.subBuses["dynamics"].subBuses["posVel"].signals["vel_I"].update(orbitObj.vel_I)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["sma"].update(orbitObj.keplerElem.sma)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["ecc"].update(orbitObj.keplerElem.ecc)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["inc"].update(orbitObj.keplerElem.inc)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["raan"].update(orbitObj.keplerElem.raan)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["argPer"].update(orbitObj.keplerElem.argPer)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["ta"].update(orbitObj.keplerElem.ta)

    return modelsBusOut


def setInitialOrbit(modelsBus, orbitInitParam):
    # Initialize outputs bus
    modelsBusOut = modelsBus
    perAltitudeInit = orbitInitParam.perAltitudeInit
    ecc = orbitInitParam.ecc
    inc = orbitInitParam.inc
    raan = orbitInitParam.raan
    argPer = orbitInitParam.argPer
    ta = orbitInitParam.ta

    # Compute initial orbit
    perRadiusInit = perAltitudeInit + const.earthRadius
    sma = perRadiusInit / (1-ecc)
    (pos_I, vel_I) = elementsToPosVelInertial(sma, ecc, inc, raan, argPer, ta, const.earthMu)
    orbit = Orbit(const.earthMu, pos_I, vel_I)

    # Set output bus signals
    modelsBusOut.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].update(orbit.pos_I)
    modelsBusOut.subBuses["dynamics"].subBuses["posVel"].signals["vel_I"].update(orbit.vel_I)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["sma"].update(orbit.keplerElem.sma)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["ecc"].update(orbit.keplerElem.ecc)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["inc"].update(orbit.keplerElem.inc)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["raan"].update(orbit.keplerElem.raan)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["argPer"].update(orbit.keplerElem.argPer)
    modelsBusOut.subBuses["dynamics"].subBuses["orbitElem"].signals["ta"].update(orbit.keplerElem.ta)

    return (modelsBusOut, orbit)


# Intertial position/velocity to orbit elements
def posVelInertialToElements(posVec, velVec, mu):
    # Pre-processing
    eps = 1e-10
    posVec = np.array(posVec);
    velVec = np.array(velVec)

    # Default outputs
    isEquatorial = True
    isCircular   = True
    ecc = 0
    inc = 0
    raan = 0
    argPer = 0
    taEp = 0    
    argLat = 0
    lonPer = 0    
    lonEp = 0
    
    # Position radius and velocity amplitude
    r = np.linalg.norm(posVec)
    v = np.linalg.norm(velVec)

    dot_posVel = np.dot(posVec, velVec)

    # Angular momentum
    angMomVec = np.cross(posVec,velVec)
    h = np.linalg.norm(angMomVec)
    # Node vector
    nodeVec = np.cross(np.array([0,0,1]),angMomVec)
    n = np.linalg.norm(nodeVec)
    if np.abs(n)>eps:
        isEquatorial = False
    # Eccentricity
    eccVec  = 1/mu*((v**2-mu/r)*posVec-np.dot(posVec,velVec)*velVec)
    ecc     = np.linalg.norm(eccVec)
    if np.abs(ecc)>eps:
        isCircular = False
    # Semilatus rectum
    p = h**2/mu        
    # radius of perigee/apogee
    perRad = p/(1+ecc)
    apoRad = p/(1-ecc)
    # sema major axis
    # sma = 1/2*(perRad+apoRad)
    sma = (h/np.sqrt(mu*(1-ecc**2)))**2
    
    # Inclination 
    if not(isEquatorial):
        cosInc = angMomVec[2]/h
        inc = np.arccos(cosInc)     
    # Right ascension of ascending node
    if not(isEquatorial):
        cosRaan = nodeVec[0]/n
        raan = np.arccos(cosRaan)
    # Argument of periapsis 
    if not(isCircular) and not(isEquatorial):
        cosArgPer = np.dot(nodeVec,eccVec)/(n*ecc)
        argPer = np.arccos(cosArgPer)
    # True anomaly at epoch
    if not(isCircular):
        cosTaEp = np.dot(eccVec,posVec)/(ecc*r)
        if (dot_posVel>0):
            taEp = np.arccos(cosTaEp)
        else:
            taEp = 2*np.pi - np.arccos(cosTaEp)
    # Longitude of periapsis
    if not(isCircular):
        cosLonPer = eccVec[0]/(ecc)
        lonPer = np.arccos(cosLonPer)
    # Longitude of epoch
    lonEp = lonPer + taEp        
        
    return (sma ,ecc, inc, raan, argPer, taEp, lonPer, lonEp, perRad, apoRad, angMomVec)


# DCM between perifocal and inertial
def dcm_perifocalToInertial(raan, inc, argPer):
    # pre processing
    # default outputs
    M_if = np.eye(3)
    
    # computation of rotation elements
    M_if[0,0] = +np.cos(raan)*np.cos(argPer) - np.sin(raan)*np.sin(argPer)*np.cos(inc)
    M_if[1,0] = +np.sin(raan)*np.cos(argPer) + np.cos(raan)*np.sin(argPer)*np.cos(inc)
    M_if[2,0] = +np.sin(argPer)*np.sin(inc)
    
    M_if[0,1] = -np.cos(raan)*np.sin(argPer) - np.sin(raan)*np.cos(argPer)*np.cos(inc)
    M_if[1,1] = -np.sin(raan)*np.sin(argPer) + np.cos(raan)*np.cos(argPer)*np.cos(inc)
    M_if[2,1] = +np.cos(argPer)*np.sin(inc)    
    
    M_if[0,2] = +np.sin(raan)*np.sin(inc)    
    M_if[1,2] = -np.cos(raan)*np.sin(inc)
    M_if[2,2] = np.cos(inc)
    
    return M_if


# DCM between inertial and geocentric
def dcm_InertialToGeocentric(inputs):
    # pre processing
    # default outputs
    M_ei = np.eye(3)
    return M_ei


# Position in geocentric to geocentric latitude/longitude
def geocentricToLatLon(pos_e):
    # Pre-processing
    # Default outputs
    lat = 0
    lon = 0
    # Compute geocentric latitude, longitude
    lon = np.arctan2(pos_e[1], pos_e[0])
    lat = np.arcsin(pos_e[2] / np.sqrt(pos_e[0]**2 + pos_e[1]**2))
    return(lat, lon)


# Orbit elements to perifocal position/velocity
def elementsToPosVelPerifocal(sma, ecc, inc, raan, argPer, ta, mu):
    # pre processing
    X_f = np.array([1,0,0])
    Y_f = np.array([0,1,0])
    # default outputs
    posVec = np.array([0,0,0])
    velVec = np.array([0,0,0])
    
    # semilatus rectum
    p = sma * (1 - ecc**2)
    # current radius
    r = p / (1 + ecc*np.cos(ta))
    # position vector in perifocal frame
    posVec_f = r * np.cos(ta) * X_f + r * np.sin(ta) * Y_f
    # velocity vector in perifocal frame
    velVec_f = np.sqrt(mu/p) * (-np.sin(ta) * X_f + (ecc + np.cos(ta)) * Y_f)
    
    return (posVec_f,velVec_f) 


# Orbit elements to inertial position/velocity
def elementsToPosVelInertial(sma, ecc, inc, raan, argPer, ta, mu):
    # Default outputs
    posVec_i = np.array([0,0,0])
    velVec_i = np.array([0,0,0])
    # Compute pos/vel in perifocal frame
    (posVec_f, velVec_f) = elementsToPosVelPerifocal(sma, ecc, inc, raan, argPer, ta, mu)
    # Compute transformation matrix
    M_if = dcm_perifocalToInertial(raan,inc,argPer)
    # Compute pos/vel in inertial frame
    posVec_i = np.matmul(M_if, posVec_f)
    velVec_i = np.matmul(M_if, velVec_f)
    # Outputs
    return(posVec_i, velVec_i)
   
# Compute orbit rate
def getOrbitRate(mu, sma):
    orbitRate = np.sqrt(mu / pow(sma, 3))
    return orbitRate


# --------------------------------------------------
# POSITION AND VELOCITY DYNAMICS
# --------------------------------------------------

def posDot(posPrev, velPrev):
    return velPrev

def velDot(posPrev, velPrev, mu):
    posRadius = np.linalg.norm(np.array(posPrev))    
    velDot = -mu/posRadius**3 * np.array(posPrev)
    return velDot

def dotPosVel(xPrev, envPrev, mu):
    (pos_I, vel_I) = vecToPosVel(xPrev)
    dotPos_I = posDot(pos_I, vel_I)
    dotVel_I = velDot(pos_I, vel_I, mu)
    return np.concatenate((dotPos_I, dotVel_I), 0)

def vecToPosVel(vec):
    pos = vec[0:3]
    vel = vec[3:6]
    return (pos, vel)

# --------------------------------------------------
# SIMULATION RUN FUNCTIONS
# --------------------------------------------------

def runOrbitDynamicsSim(posStart,velStart,Ts,Tend,mu):
    print("starting runOrbitDynamicsSim...")
    tStart = ti.time()
    # 
    nPoints = np.int(Tend/Ts+1)
    # initialize propagation data
    posPrev = posStart
    velPrev = velStart
    # compute additional data at start time
    pos_e = np.matmul(dcm_InertialToGeocentric(0),posStart)
    vel_e = np.matmul(dcm_InertialToGeocentric(0),velStart)
    (lat_e,lon_e) = geocentricToLatLon(pos_e)
    (sma,ecc,inc,raan,argPer,taEp,lonPer,lonEp,perRad,apoRad,angMomVec) = posVelInertialToElements(posStart, velStart, mu)    
    # create time vectors
    timePos_i = np.zeros((nPoints,3))
    timeVel_i = np.zeros((nPoints,3))
    timePos_e = np.zeros((nPoints,3))
    timeVel_e = np.zeros((nPoints,3))
    timeLat   = np.zeros((nPoints,1))
    timeLon   = np.zeros((nPoints,1))
    time      = np.zeros((nPoints,1))
    timeSma    = np.zeros((nPoints,1))
    timeEcc    = np.zeros((nPoints,1))
    timeInc    = np.zeros((nPoints,1))
    timeRaan   = np.zeros((nPoints,1))
    timeArgPer = np.zeros((nPoints,1))
    timeApoRad = np.zeros((nPoints,1))
    timePerRad = np.zeros((nPoints,1))
    timeAngMom = np.zeros((nPoints,3))
    # initialize first element of time vectors
    timePos_i[0,:] = posStart
    timeVel_i[0,:] = velStart
    timePos_e[0,:] = pos_e
    timeVel_e[0,:] = vel_e
    timeLat[0] = lat_e
    timeLon[0] = lon_e
    timeSma[0] = sma
    timeEcc[0] = ecc
    timeInc[0] = inc
    timeRaan[0]   = raan
    timeArgPer[0] = argPer
    timeApoRad[0] = apoRad
    timePerRad[0] = perRad
    timeAngMom[0,:] = angMomVec
    
    for ii in range(1,nPoints):    
        # propagate position and velocity
        (posNext,velNext) = propagatePosVel(posPrev, velPrev, 0, Ts, mu)
        # compute additional data
        pos_e = np.matmul(dcm_InertialToGeocentric(0),posNext)
        vel_e = np.matmul(dcm_InertialToGeocentric(0),velNext)
        (lat_e,lon_e) = geocentricToLatLon(pos_e)
        (sma,ecc,inc,raan,argPer,taEp,lonPer,lonEp,perRad,apoRad,angMomVec) = posVelInertialToElements(posNext, velNext, mu)
        # fill time vectors
        time[ii]      = ii*Ts
        timePos_i[ii,:] = posNext
        timeVel_i[ii,:] = velNext 
        timePos_e[ii,:] = pos_e
        timeVel_e[ii,:] = vel_e
        timeLat[ii] = lat_e
        timeLon[ii] = lon_e
        timeSma[ii] = sma
        timeEcc[ii] = ecc
        timeInc[ii] = inc
        timeRaan[ii]   = raan
        timeArgPer[ii] = argPer     
        timeApoRad[ii] = apoRad
        timePerRad[ii] = perRad
        timeAngMom[ii,:] = angMomVec
        # prepare next step
        posPrev = posNext
        velPrev = velNext
    # 
    tEnd = ti.time()
    print("finished. elapsed time is %.2f seconds" %(tEnd-tStart))
    return (time,timePos_i,timeVel_i,timePos_e,timeVel_e,timeLat,timeLon,timeSma,timeEcc,timeInc,timeRaan,timeArgPer,timeApoRad,timePerRad,timeAngMom)


# --------------------------------------------------
# POST PROCESSING FUNCTIONS
# --------------------------------------------------

# compute norm of vector time data
def computeNorm(timeData):
    nPoints = len(timeData)
    timeDataNorm = np.zeros((nPoints,1))
    for ii in range(nPoints):
        timeDataNorm[ii,0] = np.linalg.norm(timeData[ii,:])
        
    return timeDataNorm


# display latitude/longitude
def plotLatLon(time,timeLat,timeLon):
    figLatLonTime = plt.figure("LatLonTime")
    plt.plot(time,timeLat/deg2rad, label="lat", color="blue")
    plt.plot(time,timeLon/deg2rad, label="lon", color="red")
    plt.grid()
    plt.title("Latitude/longitude in geocentric frame")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="upper left")
    # plt.show()
    
    figLatLonProj = plt.figure("LatLonProj")
    plt.scatter(timeLon/deg2rad,timeLat/deg2rad,label="trace",color="blue")
    plt.grid()
    plt.title("Orbit trace")
    plt.xlabel("Lon [deg]")
    plt.ylabel("Lat [deg]")
    plt.legend(loc="upper left")
    # plt.show()
    
    return(figLatLonTime,figLatLonProj)
    

# display position coordinates
def plotPosition(time,timePos,timePos_e):
    # inertial position
    figPosCoord, axs = plt.subplots(1,2)
    figPosCoord.canvas.manager.set_window_title("PosCoord")
    axs[0].plot(time,timePos[:,0]*10**-3, label="x", color="blue")
    axs[0].plot(time,timePos[:,1]*10**-3, label="y", color="red")
    axs[0].plot(time,timePos[:,2]*10**-3, label="z", color="black")
    axs[0].title.set_text("Position in inertial frame")
    axs[0].set_xlabel("Time [s]")
    axs[0].set_ylabel("Pos [km]")
    axs[0].grid()
    
    axs[1].plot(time,timePos_e[:,0]*10**-3, label="x", color="blue")
    axs[1].plot(time,timePos_e[:,1]*10**-3, label="y", color="red")
    axs[1].plot(time,timePos_e[:,2]*10**-3, label="z", color="black")
    axs[1].title.set_text("Position in geocentric frame")
    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Pos [km]")
    axs[1].grid()
    # plt.show()     
    return(figPosCoord)


# display elements
def plotElements(time,timeSma,timeEcc, timeInc, timeRaan, timeArgPer, timeApoRad, timePerRad, timeAngMom):
    # inertial position
    figSmaEcc, axs = plt.subplots(1,2)
    figSmaEcc.tight_layout()
    figSmaEcc.canvas.manager.set_window_title("SmaEcc")    
    axs[0].plot(time,timeSma*10**-3, label="Sma", color="blue")
    axs[0].set_ylabel("Sma [km]")
    axs[0].set_xlabel("Time [s]")
    axs[0].grid()
    axs[1].plot(time,timeEcc, label="Ecc", color="blue")
    axs[1].set_ylabel("Ecc")
    axs[1].set_xlabel("Time [s]")    
    axs[1].grid()
    # plt.show()
    
    # incm raan, arg of perigee
    figAngles, axs = plt.subplots(3,1)
    figAngles.tight_layout()
    figAngles.canvas.manager.set_window_title("Angles")      
    axs[0].plot(time,timeInc/deg2rad,label="Inc",color="blue")
    axs[0].set_ylabel("Inc [deg]")
    axs[0].set_xlabel("Time [s]")
    axs[0].grid()    
    axs[1].plot(time,timeRaan/deg2rad,label="Raan",color="blue")
    axs[1].set_ylabel("Raan [deg]")
    axs[1].set_xlabel("Time [s]")
    axs[1].grid()    
    axs[2].plot(time,timeArgPer/deg2rad,label="ArgPer",color="blue")
    axs[2].set_ylabel("ArgPer [deg]")
    axs[2].set_xlabel("Time [s]")    
    axs[2].grid()    

    # apo/per radii
    figApoPer, axs = plt.subplots(2,1)
    figApoPer.tight_layout()
    figApoPer.canvas.manager.set_window_title("ApoPer")      
    axs[0].plot(time,timeApoRad*10**-3, label="apoRad", color="blue")
    axs[0].set_ylabel("apoRad [km]")
    axs[0].set_xlabel("Time [s]")
    axs[0].grid()
    axs[1].plot(time,timePerRad*10**-3, label="perRad", color="blue")
    axs[1].set_ylabel("perRad [km]")
    axs[1].set_xlabel("Time [s]")    
    axs[1].grid()
    # plt.show()
    
    # angular momentum
    figAngMom = plt.figure("AngMom")
    figAngMom.canvas.manager.set_window_title("AngMom")     
    plt.plot(time,timeAngMom[:,0], label="x", color="blue")
    plt.plot(time,timeAngMom[:,1], label="y", color="red")
    plt.plot(time,timeAngMom[:,2], label="z", color="black")
    plt.grid()
    plt.title("Angular momentum in inertial frame")
    plt.xlabel("Time [s]")
    plt.ylabel("Ang Mom [TBC]")
    plt.legend(loc="upper left")
    # plt.show()
    
    return(figSmaEcc,figAngles,figApoPer,figAngMom)


# display orbit
# TBW: errors when importing cartopy and launching from windows command prompt
# def plotOrbit(timePos,time):
#     figPos3d = plt.figure(figsize=(12,12))
    
#     ax = plt.axes(projection=ccrs.Orthographic(0, 0))
    
#     ax.add_feature(cartopy.feature.OCEAN, zorder=0)
#     ax.add_feature(cartopy.feature.LAND, zorder=0, edgecolor='black')
    
#     ax.set_global()
#     ax.gridlines()
    
    # ax1 = plt.axes(projection='3d')
    # ax1.plot3D(timePos[:,0]/10**3,timePos[:,1]/10**3,timePos[:,2]/10**3)
    # ax1.set_xlabel('x')
    # ax1.set_ylabel('y')
    # ax1.set_zlabel('z')
    # plt.xlim([-8000,8000])
    # plt.ylim([-8000,8000])    

    
    # x, y, u, v, vector_crs = sample_data()
    # ax.quiver(x, y, u, v, transform=vector_crs)
    
    # plt.show()        
    
    # fig = plt.figure()
    # plt.plot(time,timePos[:,0]/10**3)
    # plt.plot(time,timePos[:,1]/10**3)
    # plt.plot(time,timePos[:,2]/10**3)