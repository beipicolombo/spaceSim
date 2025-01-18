# -*- coding: utf-8 -*-
"""
Created on Fri Jul  7 00:11:31 2023

@author: Xela
"""

# import pygame module in this program
import pygame
import numpy as np
import time
import sys
import queue


class DataTextDisplay:    
    def __init__(self,nbElem,xStart,yStart,prefix,digitRes,font,frontColor,backColor):
        self.nbElem = nbElem
        self.posStart = (xStart,yStart)
        self.prefix = prefix
        self.digitRes = digitRes
        self.font = font
        self.frontColor = frontColor
        self.backColor = backColor
    
    def toText(self,dataArray):
        textList = []
        locationList = []
        
        for ii in range(self.nbElem):
            dataElem = dataArray[ii]
            prefixElem = self.prefix[ii]
            # 
            locationList.append((self.posStart[0] , self.posStart[1]+ii*20))
            # 
            strTest = "{:."+str(self.digitRes)+"f}"
            dataStr = strTest.format(dataElem)
            text = self.font.render(prefixElem+dataStr, True, self.frontColor, self.backColor) 
            #
            textList.append(text)          
        
        return (textList,locationList)
    
    def update(self,dataArray,display_surface):
        (textList,locationList) = self.toText(dataArray)
        
        for ii in range(self.nbElem):              
            display_surface.blit(textList[ii],locationList[ii])

        
def updateAttitudeIndicator(screen,screenSize,ATTITUDE_data,ATTITUDE_INDIC_font):
    # Retrieve angles of interest
    rollAngle = ATTITUDE_data[0]
    pitchAngle = ATTITUDE_data[1]
    # Retrieve screen size
    (X, Y) = screenSize
    # [TBC: use global variables] Initialize colors
    GREEN = (0, 255, 0)
    WHITE = (255, 255, 255)
    
    drawAttitudeIndicatorBackground(screen,screenSize,ATTITUDE_INDIC_font)
    drawAttitudeIndicatorRollTick(screen,screenSize,30*np.pi/180-rollAngle,40,ATTITUDE_INDIC_font) # +30deg tick
    drawAttitudeIndicatorRollTick(screen,screenSize,60*np.pi/180-rollAngle,40,ATTITUDE_INDIC_font) # +60deg tick
    drawAttitudeIndicatorRollTick(screen,screenSize,-30*np.pi/180-rollAngle,40,ATTITUDE_INDIC_font) # -30deg tick
    drawAttitudeIndicatorRollTick(screen,screenSize,-60*np.pi/180-rollAngle,40,ATTITUDE_INDIC_font) # -60deg tick
    drawAttitudeIndicatorRollTick(screen,screenSize,10*np.pi/180-rollAngle,20,ATTITUDE_INDIC_font) # +10deg tick
    drawAttitudeIndicatorRollTick(screen,screenSize,20*np.pi/180-rollAngle,20,ATTITUDE_INDIC_font) # +20deg tick
    drawAttitudeIndicatorRollTick(screen,screenSize,-10*np.pi/180-rollAngle,20,ATTITUDE_INDIC_font) # -10deg tick
    drawAttitudeIndicatorRollTick(screen,screenSize,-20*np.pi/180-rollAngle,20,ATTITUDE_INDIC_font) # -20deg tick
    drawAttitudeIndicatorHorizon(screen,screenSize,rollAngle,pitchAngle,ATTITUDE_INDIC_font)

    xTop = X//2+110*np.sin(-rollAngle)
    yTop = Y//2-110*np.cos(-rollAngle)
    xMid = xTop+20*np.sin(-rollAngle)
    yMid = yTop-20*np.cos(-rollAngle)
    deltaX = 5 * np.cos(-rollAngle)
    deltaY = 5 * np.sin(-rollAngle)
    pygame.draw.polygon(screen, WHITE,[(xTop, yTop), (xMid+deltaX, yMid+deltaY), (xMid-deltaX, yMid-deltaY)])    
    

def drawAttitudeIndicatorHorizon(screen,screenSize,rollAngle,pitchAngle,ATTITUDE_INDIC_font):
    # Retrieve screen size
    (X, Y) = screenSize
    # [TBC: use global variables] Initialize colors
    GREEN = (0, 255, 0)
    
    if abs(pitchAngle) < 40*np.pi/180:

        pitchAmplitude = -pitchAngle*160/(40*np.pi/180)
        horizLineLength = round(2*np.sqrt(pow(160,2)-pow(pitchAmplitude,2)))
        
        # Horizon line
        xMid = X//2+pitchAmplitude*np.sin(-rollAngle)
        yMid = Y//2-pitchAmplitude*np.cos(-rollAngle) 
        deltaX = horizLineLength/2 * np.cos(-rollAngle)
        deltaY = horizLineLength/2 * np.sin(-rollAngle)
        xStr = xMid-deltaX
        yStr = yMid-deltaY
        xEnd = xMid + deltaX
        yEnd = yMid + deltaY
        pygame.draw.line(screen,GREEN,(xStr,yStr),(xEnd,yEnd))  



def drawAttitudeIndicatorRollTick(screen,screenSize,rollAngle,tickLength,ATTITUDE_INDIC_font):
    WHITE = (255,255,255)
    BLACK = (0,0,0)
    # Retrieve screen size
    (X, Y) = screenSize 
    
    xStr = X//2+110*np.sin(rollAngle)
    yStr = Y//2-110*np.cos(rollAngle)
    xEnd = X//2+(110+tickLength)*np.sin(rollAngle)
    yEnd = Y//2-(110+tickLength)*np.cos(rollAngle)
    pygame.draw.line(screen,WHITE,(xStr,yStr),(xEnd,yEnd))
    


def drawAttitudeIndicatorBackground(screen,screenSize,ATTITUDE_INDIC_font):
    WHITE = (255,255,255)
    BLACK = (0,0,0)
    # Retrieve screen size
    (X, Y) = screenSize    
    
    pygame.draw.circle(screen, WHITE, (X//2,Y//2), 160, 1)
    pygame.draw.circle(screen, WHITE, (X//2,Y//2), 110, 1)
    pygame.draw.line(screen,WHITE,(X//2-80,Y//2),(X//2-40,Y//2))
    pygame.draw.line(screen,WHITE,(X//2+40,Y//2),(X//2+80,Y//2))
    pygame.draw.line(screen,WHITE,(X//2-40,Y//2+20),(X//2-40,Y//2)) 
    pygame.draw.line(screen,WHITE,(X//2+40,Y//2+20),(X//2+40,Y//2))
    pygame.draw.line(screen,WHITE,(X//2-40,Y//2+20),(X//2+40,Y//2+20))
    
    pygame.draw.polygon(screen, WHITE,[(X//2-5, Y//2-90), (X//2+5, Y//2-90), (X//2, Y//2-110)],2)    

    pygame.draw.line(screen,WHITE,(X//2-10,Y//2-20),(X//2+10,Y//2-20)) # +05 deg line
    pygame.draw.line(screen,WHITE,(X//2-20,Y//2-40),(X//2+20,Y//2-40)) # +10 deg line
    pygame.draw.line(screen,WHITE,(X//2-10,Y//2-60),(X//2+10,Y//2-60)) # +15 deg line
    pygame.draw.line(screen,WHITE,(X//2-20,Y//2-80),(X//2+20,Y//2 -80)) # +20 deg line
    pygame.draw.line(screen,WHITE,(X//2-10,Y//2+20),(X//2+10,Y//2+20)) # -05 deg line
    pygame.draw.line(screen,WHITE,(X//2-20,Y//2+40),(X//2+20,Y//2+40)) # -10 deg line
    pygame.draw.line(screen,WHITE,(X//2-10,Y//2+60),(X//2+10,Y//2+60)) # -15 deg line
    pygame.draw.line(screen,WHITE,(X//2-20,Y//2+80),(X//2+20,Y//2+80)) # -20 deg line
    
    screen.blit(ATTITUDE_INDIC_font.render(str(10), True, WHITE, BLACK),(X//2+20,Y//2-40-5))
    screen.blit(ATTITUDE_INDIC_font.render(str(20), True, WHITE, BLACK),(X//2+30,Y//2-80-5))
    screen.blit(ATTITUDE_INDIC_font.render(str(10), True, WHITE, BLACK),(X//2+20,Y//2+40-5))
    screen.blit(ATTITUDE_INDIC_font.render(str(20), True, WHITE, BLACK),(X//2+30,Y//2+80-5))    


def runGui(in_q,qStop):
    # Initialization
    # =================================================
    # Activate the pygame library
    pygame.init()

    # Set the pygame window name
    pygame.display.set_caption('FlightSimulator')    

    # Define the RGB values
    WHITE = (255, 255, 255)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 128)
    BLACK = (0, 0, 0)
 
    # Create the display surface object of specific dimension..e(X, Y).
    X = 800
    Y = 400
    windowSize = (X, Y)
    screen = pygame.display.set_mode((X, Y))
 
    # Initialize the data objects to display
    # =================================================
    TEXT_font = pygame.font.Font('freesansbold.ttf', 20)    
    ATTITUDE_INDIC_font = pygame.font.Font('freesansbold.ttf', 15) 
    # Attitude
    ATTITUDE_TEXT_prefix = ["roll (X)    : ","pitch (Y) : ","yaw (Z)   : "]    
    ATTITUDE_TEXT_digitRes = 2
    ATTITUDE_TEXT_DISP = DataTextDisplay(3,0,0,ATTITUDE_TEXT_prefix,ATTITUDE_TEXT_digitRes,TEXT_font,GREEN,BLUE)
    # Velocity
    VELOCITY_TEXT_prefix = ["(X) : ","(Y) : ","(Z) : "]    
    VELOCITY_TEXT_digitRes = 2
    VELOCITY_TEXT_DISP = DataTextDisplay(3,0,70,VELOCITY_TEXT_prefix,VELOCITY_TEXT_digitRes,TEXT_font,GREEN,BLUE)
    # NED position
    POSNED_TEXT_prefix = ["(X) : ","(Y) : ","(Z) : "]    
    POSNED_TEXT_digitRes = 2
    POSNED_TEXT_DISP = DataTextDisplay(3,0,140,POSNED_TEXT_prefix,POSNED_TEXT_digitRes,TEXT_font,GREEN,BLUE)
    
    # [TBC] Initialize background and tests
    screen.fill(BLACK)
    drawAttitudeIndicatorBackground(screen,(X,Y),ATTITUDE_INDIC_font) 
    updateAttitudeIndicator(screen,(X,Y),np.array([0,0,0]),ATTITUDE_INDIC_font)

    pygame.display.update() 
    
    while True:
        # White background
        screen.fill(BLACK)
        drawAttitudeIndicatorBackground(screen,(X,Y),ATTITUDE_INDIC_font)

        # Retrieve data from queue
        if not in_q.empty():
            dataCur = in_q.get()     
            ATTITUDE_data = dataCur[0]
            VELOCITY_data = dataCur[1]*3600/1000
            POSNED_data = dataCur[2]/1000
            
            sys.stdout.flush()
            
            # Update text display states
            ATTITUDE_TEXT_DISP.update(ATTITUDE_data*180/np.pi,screen)                  
            VELOCITY_TEXT_DISP.update(VELOCITY_data,screen)    
            POSNED_TEXT_DISP.update(POSNED_data,screen)    
            
            # Update attitude indicator state
            updateAttitudeIndicator(screen,(X,Y),ATTITUDE_data,ATTITUDE_INDIC_font)
            # drawAttitudeIndicatorPitch  
            
            # Update display
            pygame.display.update()            
            
       
        # Iterate over the list of Event objects that was returned by pygame.event.get() method.
        for event in pygame.event.get():
            # If event object type is QUIT then quitting the pygame and program both.
            if ((event.type == pygame.QUIT) or (event.type == pygame.K_ESCAPE)):     
                # Deactivates the pygame library
                isStopped = True
                qStop.put(isStopped)
                print('GUI thread stopped') 
                pygame.quit()
                sys.exit(0)
     
        # pygame.time.delay(1000)
        sys.stdout.flush()
        time.sleep(0.05)
        