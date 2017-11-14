import vision.obstacledetector
from navigation.obstacleavoidance.vectorfield import VectorField
import sys
import math
import time

class ObstacleAvoider:
    """ avoid obstacles using potential field navigation
        NOTE: The magic (calculating the final direction using the obstacle matrix)
                happens in the vectorfield.py module.
    """

    def __init__(self, standalone=False, detectorsource="kinect"):
        self.vectorField = VectorField()

        # if the obstacle avoider is run a standalone module,
        # it will get the (scanned) obstacle matrix directly from the
        # obstacle detector instead of from the memory
        self.standalone = standalone

        # initally don't use visualization.
        # enable it with 'startVisualization();
        self.visualize = 0

        self.total_frames = 0
        self.total_delay = 0
        self.total_arch = 0
        self.total_network = 0
        self.total_local = 0

        if standalone == True:
            import vision.obstacledetector
            if detectorsource != "kinect":
                self.obstacleDetector = vision.obstacledetector.ObstacleDetector(standalone=True, source='file', filepath=detectorsource)
            else:
                self.obstacleDetector = vision.obstacledetector.ObstacleDetector(standalone=True, source='kinect')
        else:
            import memory
            self.m = memory.Memory()

        if self.visualize == 1:
            self.startVisualization()

    def do_visualize(self):
        # update visualizer (if enabled)
        if self.visualize == 1:
            last_observation = self.m.get_last_observation('obstacle_matrix')
            if (last_observation != None):
                completeObstacleMatrix = last_observation[1]['matrix']
                scannedObstacleMatrix = self.getScannedObstacleMatrix(completeObstacleMatrix)
		self.visualizer.setObstacleMatrix(completeObstacleMatrix)
                self.visualizer.setObstacleEdgesMatrix(scannedObstacleMatrix)
		self.visualizer.draw()

    def startVisualization(self):
        """
            Start the visualization module to show the desicions of the robot in real time
        """
        self.visualize = 1
        import visualisation.avoidancevisualizer
        self.visualizer = visualisation.avoidancevisualizer.AvoidanceVisualizer()

    def avoid(self, target):
        """
        Call this function if you want to go to some direction
        The avoider will take care of not bumping into obstacles
        Target (speed, angle) specifies the angle to the target with respect to the robot
        and the speed the robot should have when driving to the target
        """

        completeObstacleMatrix = None # normal obstacle matrix (with every obstacle in it)
        scannedObstacleMatrix = None # obstacle matrix with just the obstacles closest to the robot in it)

        framenumber = 0
        delay = 0
        if self.standalone == True:
            # get obstacle matrix directly from the obstacle detector
            completeObstacleMatrix = self.obstacleDetector.getObstacleMatrix()
        else:
            # get obstacle matrix from memory
            last_observation = self.m.get_last_observation('obstacle_matrix')
            if (last_observation != None):
                completeObstacleMatrix = last_observation[1]['matrix']

                if False: # set to True to enable profiling output
                    scantime = last_observation[1]['mtime']
                    local_delay = last_observation[1]['local_delay']
                    diff = time.time() - scantime
                    network_delay = last_observation[1]['network_delay']
                    architecture_delay = diff - local_delay - network_delay

                    print "Local delay: %10.5f Network delay: %10.5f Architecture delay: %10.5f Total delay: %10.5f" % (local_delay, network_delay, architecture_delay, diff)
                    self.total_frames += 1
                    self.total_network += network_delay
                    self.total_arch += architecture_delay
                    self.total_delay += diff
                    self.total_local += local_delay

                    net_avg = self.total_network / self.total_frames
                    arch_avg = self.total_arch / self.total_frames
                    local_avg = self.total_local / self.total_frames
                    total_avg = self.total_delay / self.total_frames

                    net_per = (net_avg / total_avg) * 100
                    arch_per = (arch_avg / total_avg) * 100
                    local_per = (local_avg / total_avg) * 100
                    print "Avg consumed: local: %10.5f (%4.1f %%) network: %10.5f (%4.1f %%) architecture: %10.5f (%4.1f %%) Total: %10.5f (100 %%)" % (local_avg, local_per, net_avg, net_per, arch_avg, arch_per, total_avg)
            else:
                print "No obstacle matrix retreived!"
                completeObstacleMatrix = None

        if completeObstacleMatrix != None:
            #convert obstacle matrix to scanned obstacle matrix
            scannedObstacleMatrix = self.getScannedObstacleMatrix(completeObstacleMatrix)
            # set the vector field obstacles
            self.vectorField.setObstacleMatrix(scannedObstacleMatrix)
            self.vectorField.setTarget(target)
        else:
            # stand still
            self.vectorField.setTarget((0,0))


        # get the endvector from the vector field
        # this endVector is the final destination of the robot
        endVector = self.vectorField.update_field()

        # if no endVector is retreived (probably no obstaclematrix defined)
        # set the target (specified by a behavior) as endvector
        if endVector == None:
            endVector = target


        # update visualizer (if enabled)
        if self.visualize == 1:
            if completeObstacleMatrix != None:
                self.visualizer.setObstacleMatrix(completeObstacleMatrix)
            if scannedObstacleMatrix != None:
                self.visualizer.setObstacleEdgesMatrix(scannedObstacleMatrix)
                self.visualizer.setObstacleVectors(self.vectorField.getAllObstacleAffectVectors(scannedObstacleMatrix))
            self.visualizer.setTargetVector(target)
            self.visualizer.setEndVector(endVector)

            # draw everything (and exit if needed)
            if self.visualizer.draw() == 'exit':
                sys.exit()

        # if speed is 0, then just return target because the robot shouldn't drive anyway
        if target[0] == 0:
            return target

        # return the endVector. This is the final direction and speed for the robot
        return endVector


    def getScannedObstacleMatrix(self, obstacleMatrix):
        """
            Get the scanned obstacle matrix, generated from the normal obstacle matrix
        """
        return getScannedObstacleMatrix(obstacleMatrix)

def getScannedObstacleMatrix(obstacleMatrix):
    """ scan the obstaclematrix (like a radar). Returning a new obstaclematrix with just the obstacles that have been scanned
        this simulates the view of the robot when it was using sonar
        Obstacles get the value 1, no obstacle gets the value 0
        
        The scanned obstacle matrix differs from the normal obstacle matrix,
        in that is only contains the obstacles that are of impartance to the avoider (i.e. obstacles behind other obstacles are removed).
    """

    # fill a matrix with al zeros. Scanned obstacles will be placed in this matrix
    scannedMatrix = [[0 for col in range(len(obstacleMatrix[0]))] for row in range(len(obstacleMatrix))]

    # the range in which obstacles should be scanned. Currently this is the entire matrix.
    # the maxLineLenght can be reduced to only let obstacle near the robot affect the avoider
    maxLineLength = int(round(math.sqrt((len(scannedMatrix[0])*len(scannedMatrix[0])) + (len(scannedMatrix)*len(scannedMatrix)))))

    # algorith for substracting only obstacles in front of other obstacles (i.e. obstacles behind other obstacles are removed)
    for lineDirection in range(-90,90): # scan 180 degrees. (since the robot looks forward only, this is the entire matrix)
        # increase the line length step by step.
        # at each step the algorithm will check if there is an obstacle at the tip of the line
        # if there is, it will be added to the scanned matrix and the lineDirection will be increased
        for lineLength in range(1,maxLineLength):
            #create vector (length,direction)
            vector = (lineLength,lineDirection)
            vectorX = round(math.sin(math.radians(vector[1])) * vector[0]) # relative x-position
            vectorY = round(math.cos(math.radians(vector[1])) * vector[0]) + 1 # relative y-position
            xpos = int(len(scannedMatrix[0])/2 + vectorX) # absolute x-position
            ypos = int(len(scannedMatrix) - vectorY) # absolute y-position

            # when the linetip is outside the matrix (no obstacle spotted) continue with new direction
            if ypos < 0 or ypos >= len(scannedMatrix):
                break
            if xpos < 0 or xpos >= len(scannedMatrix[0]):
                break

            # if an obstacle is spotted, add it to the scanned matrix. Then continue with new direction.
            if obstacleMatrix[ypos][xpos] != 0:
                scannedMatrix[ypos][xpos] = 1
                #scannedMatrix[ypos][xpos] = obstacleMatrix[ypos][xpos]
                break

    return scannedMatrix

def usage():
	print "Just call the function avoid(angle)."
        print "This function will return the angle where the robot should"
        print "actually drive to (taking in consideration the obstacles)"


if __name__ == "__main__":
    usage()

    # get a standalone obstacle avoider (not using the architecture)
    avoider = ObstacleAvoider(standalone=True)
    avoider.startVisualization()
    while True:
        print avoider.avoid((1,0))

