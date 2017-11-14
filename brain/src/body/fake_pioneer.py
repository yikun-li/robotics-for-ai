
#TODO: try to keep connection alive at all times
#TODO: create a more flexible pioneer control
#TODO: create unit test

import socket
import time
import string

from navigation.obstacleavoidance.obstacleavoider import ObstacleAvoider
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.BodyController.FakePioneer').addHandler(util.nullhandler.NullHandler())

ROS_ENABLED=False

class FakePioneer(object):
    """
    Controls a pioneer robot.
    (requires the motionController to run on the pioneer)
    """

    def __init__(self, ip_address, port):
        self.logger = logging.getLogger('Borg.Brain.BodyController.FakePioneer')
        self.__ip_address = ip_address
        self.__port = int(port)
        self.logger.info("Connecting to %s:%d" % (self.__ip_address, self.__port))
        self.emergency = False

    def get_ip_address(self):
        """
        Returns the IP address as specified in the constructor.
        """
        return self.__ip_address

    def get_port(self):
        """
        Returns the port as specified in the constructor.
        """
        return self.__port
    
    def stop(self):
        pass

    def head_direction(self, target, useAvoidance=False, verbose=False, turnSpeed=1, door = False):
        """
        Set the target where the pioneer should go to.
        format = (speed, angle)
            speed in range 0 to 1
            positive values are to the right
            angle in range -180 to 180, where +-180 is backwards and 0 is forward
        This is the function behaviors should call when they want the robot
        to go to some direction.
        """

        endVector = target

        speedLeft, speedRight = self.target_to_left_right_speeds(endVector)

        self.logger.debug("endVector: %s" % repr(endVector))
        self.logger.debug("Wheel speeds: %d, %d" % (speedLeft, speedRight))

    def set_target(self, target, useAvoidance=False, verbose=False):
        """
        deprecated function.
        use head_direction instead
        """
        self.logger.info("Deprecated function set_target called. Please call head_direction.")
        self.head_direction(self, target, useAvoidance, verbose)

    def target_to_left_right_speeds(self, target):
        """
        Convert an target vector to left and right wheel speeds.
        This function should be optimized
        """

        angle = target[1] # direction in range of -90 to 90
        speed = target[0] # speed in range [0, 1]
        maxSpeed = 300 * speed
        
        left = 100 # percent
        right = 100 # percent

        # inhibit correct wheel according to the angle of the target
        if angle < 0:
            left += angle
        else:
            right -= angle

        # avoid to fast spinning when target is behind robot
        if abs(angle) > 90:
            maxSpeed /= 2

        # scale leftright speeds to maxspeed
        left = maxSpeed * (left/100.0)
        right = maxSpeed * (right/100.0)

        return (left, right)

    def set_left_right_speeds(self, speedLeft, speedRight):
        """
        Send command to set left and right wheel speed
        This is the function behaviors used to call to move the robot.
        Do NOT use it anymore.
        Behaviors should call set_target(target)
        """
        self.logger.debug("Speed as received %d, %d" % (speedLeft, speedRight))

###
    def forward(self, distance):
        """
        Forward in mm. Take care, this will continue until receives a new command or is finished
        DOES NOT USE OBSTACLE AVOIDANCE!
        """
        self.logger.debug("forward " + str(distance))

    def turn(self, angle):
        """
        Turn in degrees. Take care, this will continue until receives a new command or is finished
        left is positive
        DOES NOT USE OBSTACLE AVOIDANCE!
        """
        self.logger.debug("turn " + str(angle))
        
    def stop_robot(self):
        ''' Send command to stop the robot'''
        self.logger.debug("stop\n")
        
    # The Functions to read odometry from the pioneer 
    def update(self):
        """Get update from all speech modules"""
        self.detections = []
        return self.detections  
