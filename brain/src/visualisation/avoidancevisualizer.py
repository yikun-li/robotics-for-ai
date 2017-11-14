import pygame
import cv
import math

import vision.obstacledetectorutil.segmentizer

from pygame.locals import *
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.Visualization.AvoidanceVisualizer').addHandler(util.nullhandler.NullHandler())

class AvoidanceVisualizer:
    """ visualize the obstacle detection and avoidance """

    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.Visualization.AvoidanceVisualizer')
        self.edgesMatrix = 0
        self.obstacleMatrix = 0
        self.obstacleVectors = 0
        self.targetVector = 0
        self.endVector = 0
        self.depthImage = 0
        self.RGBImage = 0
        
        self.region = vision.obstacledetectorutil.segmentizer.getRegion()
        self.segWidth, self.segHeight = vision.obstacledetectorutil.segmentizer.getSegSize()
        self.segsize = (self.segWidth, self.segHeight)

        # initialize screen
        self.screen = pygame.display.set_mode( (640, 480) )
        #initialize font
        pygame.font.init()

        self.vectorScaleFactor = 50

        self.center = (self.screen.get_width()/2,self.screen.get_height()-100) # bottom center of screen





    def draw(self):
        """ draw everything to the the screen """

        # check if the visualizer should stop
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return 'exit'
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                pygame.quit()
                return 'exit'
            elif event.type == KEYDOWN:
                return event.key

        # clear screen
        self.screen.fill((0, 0, 0))

        # draw the items
        self.drawDepthImage()
        self.drawRGBImage()
        self.drawObstacles()
        self.drawObstacleEdges()
        self.drawCross()
        self.drawRegionRect()
        self.drawTargetVector()
        self.drawObstacleVectors()
        self.drawEndVector()
        self.drawText()

        # commit changes (or something like that)
        pygame.display.flip()

        return 0

    def drawDepthImage(self):
        """ Draw the caputered image to the screen. This is very slooow, so ommit this function in competition """

        image = self.depthImage

        if image != 0:
            # make sure 8 bit image is displayed
            if image.depth == 16:
                newImage = cv.CreateImage((640,480), cv.IPL_DEPTH_8U, 1)
                cv.Convert(image, newImage)
                image = newImage


            # generate pygame image
            image_rgb = cv.CreateMat(image.height, image.width, cv.CV_8UC3)
            cv.CvtColor(image, image_rgb, cv.CV_GRAY2RGB)
            pygameImage = pygame.image.frombuffer(image_rgb.tostring(), cv.GetSize(image_rgb), "RGB")

            # draw the image
            self.screen.blit(pygameImage,(0,0))

    def drawRGBImage(self):
        """ Draw the caputered RGB image to the screen. """

        image = self.RGBImage

        if image != 0:
            # generate pygame image
            pygameImage = pygame.image.frombuffer(image.tostring(), cv.GetSize(image), "RGB")

            # draw the image
            self.screen.blit(pygameImage,(0,0))

    def drawObstacleEdges(self):
        """ draw the edges of obstacles to the screen """

        if self.edgesMatrix == 0:
            return

        for y in range(len(self.edgesMatrix)):
            for x in range(len(self.edgesMatrix[0])):
                position = (x*self.segWidth+1+self.region[0], y*self.segHeight+1+self.region[1], self.segWidth-1, self.segHeight-1)
                if self.edgesMatrix[y][x]:
                    #redvalue = max(255,(255/100) * self.edgesMatrix[y][x])
                    pygame.draw.rect(self.screen, (255, 0, 0), position, 2)
                else:
                    #pygame.draw.rect(self.screen, (0, 255, 0), position, 1)
                    pass

    def drawObstacles(self):
        """ draw the detected obstacles to the screen """

        if self.obstacleMatrix == 0:
            return

        for y in range(len(self.obstacleMatrix)):
            for x in range(len(self.obstacleMatrix[0])):
                position = (x*self.segWidth+1+self.region[0], y*self.segHeight+1+self.region[1], self.segWidth-1, self.segHeight-1)
                if self.obstacleMatrix[y][x] != 0:
                    colorvalue = min(255,(255/30) * abs(self.obstacleMatrix[y][x]))
                    if self.obstacleMatrix[y][x] > 0: # there is an obstacle
                        color = (colorvalue, 0, 0)
                    else: # there is a gap
                        color = (0, 0, colorvalue)
                    pygame.draw.rect(self.screen, color, position, 1)
                else:
                    pygame.draw.rect(self.screen, (0, 255, 0), position, 1)

    def drawCross(self):
        """ Draws an huge cross when no obstacle are profided """

        if self.obstacleMatrix == 0 and self.edgesMatrix == 0:
            width, height = self.screen.get_width(), self.screen.get_height()
            pygame.draw.line(self.screen, (255,0,0), (0,0), (width, height))
            pygame.draw.line(self.screen, (255,0,0), (width, 0), (0, height))

    def drawRegionRect(self):
        """ draw a rectangle showing the region of interest of the obstacle detecter """
        
        if self.region == 0:
            return
        
        pygame.draw.rect(self.screen, (255, 255, 255), self.region, 1)

    def drawTargetVector(self):
        """ draw the target vector to the screen """

        if self.targetVector != 0 and self.targetVector != None:
            vector = (self.targetVector[0]*self.vectorScaleFactor, self.targetVector[1])
            vectorX = math.sin(math.radians(vector[1])) * vector[0]
            vectorY = math.cos(math.radians(vector[1])) * vector[0]
            location = (self.center[0]+vectorX,self.center[1]-vectorY)
            pygame.draw.line(self.screen,(255,255,0),location, self.center)

    def drawObstacleVectors(self):
        """ draw all the obstacle vectors to the screen """

        if self.obstacleVectors != 0 and self.obstacleVectors != None:
            for vector in self.obstacleVectors:
                vector = (vector[0]*self.vectorScaleFactor, vector[1])
                vectorX = math.sin(math.radians(vector[1])) * vector[0]
                vectorY = math.cos(math.radians(vector[1])) * vector[0]
                location = (self.center[0]+vectorX,self.center[1]-vectorY)
                pygame.draw.line(self.screen,(255,0,0),location, self.center)

    def drawEndVector(self):
        """ draw the endvector to the screen """

        if self.endVector != 0:
            # convert vector (length, direction) to vector (xpos, ypos) and draw it

            vector = (self.endVector[0]*self.vectorScaleFactor, self.endVector[1])

            vectorX = math.sin(math.radians(vector[1])) * vector[0]
            vectorY = math.cos(math.radians(vector[1])) * vector[0]

            location = (self.center[0]+vectorX,self.center[1]-vectorY)

            pygame.draw.line(self.screen, (0,0,255),location, self.center, 4)

    def drawText(self):
        """ draw text that descripes what hell is going on on the screen"""

        if pygame.font:
            # font to be used
            font = pygame.font.Font(None, 20)

            # text to be written
            textRedVectors = font.render("Obstacle avoid direction", 1, (255, 0, 0))
            textYellowVector = font.render("Goal direction", 1, (255, 255, 0))
            textBlueVector = font.render("Final direction of the robot", 1, (0, 0, 255))
            textSegSize = font.render("Segment size: " + str(self.segsize), 1, (0,255,0))
            textRegionOfInterest = font.render("ROI rect: " + str(self.region), 1, (255,255,255))

            # image dimensions
            imageHeight = self.screen.get_height()
            imageWidth = self.screen.get_width()

            # draw the text
            self.screen.blit(textRedVectors, (0, imageHeight - 60))
            self.screen.blit(textYellowVector, (0, imageHeight - 40))
            self.screen.blit(textBlueVector, (0, imageHeight - 20))
            self.screen.blit(textSegSize, (imageWidth/2, imageHeight - 40))
            self.screen.blit(textRegionOfInterest, (imageWidth/2, imageHeight - 20))

    def setScreen(self, screen):
        self.screen = screen

    def setDepthImage(self, image):
        self.depthImage = image

    def setRGBImage(self, image):
        self.RGBImage = image

    def setObstacleEdgesMatrix(self, edgesMatrix):
        self.edgesMatrix = edgesMatrix

    def setObstacleMatrix(self, obstacleMatrix):
        self.obstacleMatrix = obstacleMatrix

    def setObstacleVectors(self, obstacleVectors):
        self.obstacleVectors = obstacleVectors

    def setTargetVector(self, targetVector):
        self.targetVector = targetVector

    def setEndVector(self, endVector):
        self.endVector = endVector

    def setRegionRect(self, regionRect):
        self.region = regionRect

    def setSegSize(self, segsize):
        self.segsize = segsize
        self.segWidth = segsize[0]
        self.segHeight = segsize[1]
