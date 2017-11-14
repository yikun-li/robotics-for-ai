# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="rik"
__date__ ="$May 11, 2011 10:31:57 PM$"

import logging
import util.nullhandler
from navigation.obstacleavoidance import vectorfield
from navigation.obstacleavoidance import obstacleavoider

logging.getLogger('Borg.Brain.Memory').addHandler(util.nullhandler.NullHandler())


class DoorChecker():
    """ This class provides functionallity to check if a door in front of the
        robot is open of closed
    """

    def __init__(self, standalone=False):
        # vectorvield provides 'obstacle avoider' functionallity
        # if the vectorfield says the robot cannot move forward
        self.vectorField = vectorfield.VectorField()

        # if the doorchecker is run as a standalone module,
        # it will get the (scanned) obstacle matrix directly from the
        # obstacle detector instead of from the memory
        self.standalone = standalone

        if standalone == True:
            import vision.obstacledetector
            self.obstacleDetector = vision.obstacledetector.ObstacleDetector(standalone=True)
        else:
            import memory
            self.m = memory.Memory()


        self.logger = logging.getLogger("Borg.Brain.Memory")

    def inFrontOfRobot(self):
        """ check if the door is in front of the robot
            if the robot spots the door is closed, it nows it is in front of him
            else the robot assumes the door is not in fron of him
        """

        if self.isOpen():
            return False
        else:
            return True


    def isOpen(self):
        """ check if the door in front of the robot is open """

        obstacleMatrix = self.getObstacleMatrix()

        # simulate that the robot wants to move forward
        self.vectorField.setTarget((1,0))

        # set the obstacles (a closed door would be preceived as an obstacle)
        if obstacleMatrix != None:
            self.vectorField.setObstacleMatrix(obstacleMatrix)

        # get the result of the vector field
        endVector = self.vectorField.update_field()

        # if the endVector would tell the robot to go forward, we can assume
        # the door is open
        speed, angle = endVector
        
        #print "Speed: " + str(speed) + ", angle: " + str(angle)
        
        #there should be speed
        if speed == 0:
            return False
        
        # the angleThreshold tells between which angles we can assume the robot,
        # wants to go trough the door. Is the angle greather than this threshold
        # then the robot probably wants to avoid a closed door
        #angleThreshold = 45 #degrees
        angleThreshold = 45 #degrees
        if (angle > -angleThreshold) and (angle < angleThreshold):
            return True #door is open
        else:
            return False #door is closed

        return False


    def getObstacleMatrix(self):
        """ retreive the obstacle matrix """

        completeObstacleMatrix = None

        if self.standalone == True:
            # get obstacle matrix directly from the obstacle detector
            completeObstacleMatrix = self.obstacleDetector.getObstacleMatrix()
        else:
            # get obstacle matrix from memory
            last_observation = self.m.get_last_observation('obstacle_matrix')
            if (last_observation != None):
                completeObstacleMatrix = last_observation[1]['matrix']
            else:
                self.logger.warning("No obstacle matrix retreived!")
                return None

        #return the scanned obstacle matrix
        return obstacleavoider.getScannedObstacleMatrix(completeObstacleMatrix)


if __name__ == "__main__":
    doorchecker = DoorChecker(standalone=True)
    while True:
        if doorchecker.isOpen() == True:
            print "Door is open!"
        else:
            print "closed"

