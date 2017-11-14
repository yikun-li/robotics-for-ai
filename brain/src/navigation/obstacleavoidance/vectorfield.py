import math
import vision.obstacledetectorutil.segmentizer
from vision.obstacledetectorutil.segmentizer import Segmentizer
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.NavigationController.ObstacleAvoider.VectorField').addHandler(util.nullhandler.NullHandler())

class VectorField:
    """ vectorfield with vectors influincing the robot
        set items (vectors, matrices, etc.) at will, but run update_field to apply changes

        The vectorfield will calculate the final direction and speed the robot should take to avoid obstacles
        It uses the (scanned) obstacle matrix and the target direction to do this.
        Using potential field navigation the final endVector is calculated

    """

    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.NavigationController.ObstacleAvoider.VectorField')
        
        # detector vars
        self.segSize = vision.obstacledetectorutil.segmentizer.getSegSize()
        self.region = vision.obstacledetectorutil.segmentizer.getRegion()

        # parameters

        #minimum distance percentage the obstacle vector should have to be affecting the robot
        # this is the percentage of the entire region in which obstacles are detected
        # 100 percent means that all obstacles in the entire region should be taken into account
        self.distancePercentage = 100 #percent

        
        width = self.region[2]/2 # region width/2
        height = self.region[3] # region height

        # length that a vector should have bofore taken into account
        self.obstacleDistanceThreshold = math.sqrt(width*width + height*height) * (self.distancePercentage/100.0)


        # attributes
        self.target = 0 # where the robot whants to go to
        self.obstacleMatrix = 0 #matrix telling where the obstacles are
        self.endVector = 0 # endvector to be returned


    def update_field(self):
        """ update the vector field
            returning the endVector (final speed and direction of the robot)
        """
        self.endVector = self.getEndVector(self.obstacleMatrix)

        return self.endVector



    def setObstacleMatrix(self, obstacleMatrix):
        """ Set the obstaclematrix. Call update_field() to apply changes"""
        self.obstacleMatrix = obstacleMatrix

    def setTarget(self, target):
        """ set the target where the robot would like to go to.
            target = vector (length, direction)"""
        self.target = target




    def getTarget(self):
        """ get the robot's target vector """
        return self.target



    def getEndVector(self, obstacleMatrix):
        """ get the new direction where the robot should go to.
            taking in account obstacles and the current target
            formate = (length, direction) """

        vectors = []
        if obstacleMatrix != 0:
            # each of the obstacles in the obstacle matrix will have a corresponding vector
            # 'vectors' contains these vectors
            vectors = self.getAllObstacleAffectVectors(obstacleMatrix)

        # sum alle the vectors to obtain the endVector
        # also the target direction will be taken into account
        # endvector, format (x-position, y-position)
        endVector = self.sumVectors(vectors)

        # convert to (length, direction) vector
        x = endVector[0]
        y = endVector[1]
        length = math.sqrt(x*x + y*y)

        # set max lenght to avoid outragious vectors
        length = min(length, 1)


        direction = math.degrees(math.atan2(x,y))

        return (length, direction)

    def affectVector(self, obstacleVector):
        """ Convert the obstacleVector to a affectVector which descripes the affect the obstacle has on the robot.
            This means the result vector is pointing away from the obstacle """


        vectorLength = obstacleVector[0]

        distanceThreshold = self.obstacleDistanceThreshold #minimum required distance to obstacle


        # some checks to prevent divide by zero errors
        if vectorLength == 0:
            print "Vectorfield.affectVector: vectorLength should not be 0"
            return None
        if distanceThreshold == 0:
            print "Vectorfield.affectVector: minimumLength should not be 0"
            return None

        # check if the obstacle is whithin the affect range
        if (distanceThreshold - vectorLength) < 0:
            return None

        #normalize vectorLength (0 <= length <=1)
        vectorLength = vectorLength / distanceThreshold

        # exponetionally grow the affectvector when it comes closer
        affectLength = math.pow(((1/vectorLength) - 1), 2)

        #revers vector (obstacle should be rejective in stead of attractive)
        direction = (abs(obstacleVector[1]) - 180)
        if obstacleVector[1] != 0:
            direction *= cmp(obstacleVector[1],0)

        affectVector = (affectLength, direction)
        return affectVector









    def sumVectors(self, vectors):
        """ sum all vectors (including targetvector)"""
        endObstacleVector = (0,0)

        ##generate endvector of obstacles
        #sum obstaclevectors
        for vector in vectors:
            vectorX = math.sin(math.radians(vector[1])) * vector[0] # x-position
            vectorY = math.cos(math.radians(vector[1])) * vector[0] # y-position
            endObstacleVector = (endObstacleVector[0]+vectorX,endObstacleVector[1]+vectorY)
        #mean obstaclevectors
        if len(vectors) > 0:
            endObstacleVector = (endObstacleVector[0]/len(vectors), endObstacleVector[1]/len(vectors))

        #add targetvector
        targetVector = self.target
        if targetVector != 0 and targetVector != None:
            vectorX = math.sin(math.radians(targetVector[1])) * targetVector[0] # x-position
            vectorY = math.cos(math.radians(targetVector[1])) * targetVector[0] # y-position
            endVector = (endObstacleVector[0]+vectorX,endObstacleVector[1]+vectorY)
            #endVector = (endVector[0]/2, endVector[1]/2)
        else:
            endVector = endObstacleVector


        return endVector

    def getAllObstacleAffectVectors(self, obstacleMatrix):
        """ returns all the vectors affecting the robot """
        vectors = []

        # add obstacles vectors
        height = len(self.obstacleMatrix)
        width = len(self.obstacleMatrix[0])
        for y in range(height):
            for x in range(width):
                # if there is a obstacle
                if self.obstacleMatrix[y][x] != 0:
                    # get vector from robot to obstacle
                    obstacleVector = self.getObstacleVector(x,y,width,height)
                    affectVector = self.affectVector(obstacleVector)
                    if affectVector != None:
                        vectors.append(affectVector)
        return vectors

    def getObstacleVector(self, x, y, width, height):
        """ returns a vector pointing from the robot to a obstacle given de x and y position of the obstacle
            width and height are the dimensions of the active region
            the active region is assumed to be directly in front of the robot
            vector = (length, direction)
        """
        x = (x-width/2) * self.segSize[0] #relative x
        y = (height - y) * self.segSize[1] #relative y
        length = math.sqrt( x*x + y*y ) #length of the vector
        direction = math.degrees(math.atan2(x,y)) # range between -90 (most left) and 90 (most right)
        return ( length, direction ) # location of the vector

