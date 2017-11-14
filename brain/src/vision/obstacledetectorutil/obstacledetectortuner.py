# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="rik"
__date__ ="$May 23, 2011 6:02:52 PM$"

import vision.obstacledetector

class ObstacleDetectorTuner():

    def __init__(self):
        self.detector = vision.obstacledetector.ObstacleDetector(standalone=True)

    def tune(self):

        minTolerance = 0
        maxTolerance = 0

        while 1:
            # set tollerance settings
            self.detector.setMinTolerance(minTolerance)
            self.detector.setMaxTolerance(maxTolerance)

            #receive obstacle matrix
            obstacleMatrix = self.detector.getObstacleMatrix()

            result =  self.containsObstacles(obstacleMatrix)

            # if obstacles (above ground) are found
            if result[0]:
                minTolerance += 0.1
                
            # if gaps are found
            if result[1]:
                maxTolerance += 0.1

            print "minTolerance: " + str(minTolerance)
            print "maxTolerance: " + str(maxTolerance)
            print "\n"





    def containsObstacles(self, obstacleMatrix):
        """check if obstacles are detected"""

        containsGaps = False
        containsObstacles = False
        
        for row in obstacleMatrix:
            for column in row:
                if column > 0:
                    containsObstacles = True
                if column < 0:
                    containsGaps = True

            # stop the loop if already obstacles AND gaps are found
            # no need to look futher
            if containsObstacles == True and containsGaps == True:
                break

        return (containsObstacles, containsGaps)

if __name__ == "__main__":
    tuner = ObstacleDetectorTuner()
    tuner.tune()