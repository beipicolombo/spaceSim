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

def eulerToText(eulerVec):
    # Initialize
    textList = []
    textRectList = []
    attDispCoord = [(400,400),(400,450),(400,500)]
    white = (255, 255, 255)
    green = (0, 255, 0)
    blue = (0, 0, 128)    
    
    for ii in range(0,3):
        myAng = eulerVec[ii]
        # Set text
        text = font.render(str(myAng), True, green, blue)
        textList.append(text)
        # Set text rectangle
        textRect = text.get_rect()
        (X,Y) = attDispCoord[ii]
        textRect.center = (X // 2, Y // 2)
        textRectList.append(textRect)
    
    return (textList,textRectList)

 
# activate the pygame library
# initiate pygame and give permission
# to use pygame's functionality.
pygame.init()
 
# define the RGB value for white,
#  green, blue colour .
white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)
 
# create the display surface object
# of specific dimension..e(X, Y).
X = 400
Y = 400
display_surface = pygame.display.set_mode((X, Y))
 
# set the pygame window name
pygame.display.set_caption('FlightSimulator')
 
# create a font object.
# 1st parameter is the font file
# which is present in pygame.
# 2nd parameter is size of the font
font = pygame.font.Font('freesansbold.ttf', 20)
 
 
# create a rectangular object for the
# text surface object
 
# set the center of the rectangular object.
 
eulerVec = np.array([0,0,0])

# infinite loop
while True:
 
    # completely fill the surface object
    # with white color
    display_surface.fill(white)
 
    # copying the text surface object
    # to the display surface object
    # at the center coordinate.
    # display_surface.blit(text1, textRect1)
    
    (textList,textRectList) = eulerToText(eulerVec)
    for ii in range(0,3):
        display_surface.blit(textList[ii],textRectList[ii])
    eulerVec = eulerVec + np.array([1,1,1])
    
    sys.stdout.flush()
    time.sleep(1)
    
    # iterate over the list of Event objects
    # that was returned by pygame.event.get() method.
    for event in pygame.event.get():
 
        # if event object type is QUIT
        # then quitting the pygame
        # and program both.
        if event.type == pygame.QUIT:
 
            # deactivates the pygame library
            pygame.quit()
 
            # quit the program.
            quit()
 
        # Draws the surface object to the screen.
        pygame.display.update()


