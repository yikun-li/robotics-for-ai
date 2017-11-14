#TODO: create unit test

import time
import math
import traceback

from navigation.obstacleavoidance.obstacleavoider import ObstacleAvoider
import logging
import util.nullhandler
import body.bodycontroller
from std_msgs.msg import Bool


ROS_ENABLED=False
try:
    import roslib; roslib.load_manifest('alice_msgs')
    import rospy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
    import tf
    ROS_ENABLED = True
except:
    print "[Alice.py] ROS IS NOT ENABLED"
    raise

logging.getLogger('Borg.Brain.BodyController.Alice').addHandler(util.nullhandler.NullHandler())

class Alice(object):
    """
    Controls a alice robot.
    (requires the Volksbot driver to be running)
    """

    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.BodyController.Alice')
        self.obstacleAvoider = ObstacleAvoider()
        self.emergency = False
        self.maxSpeed = 0.2
        self.doormaxSpeed = 0.5
        self.odo_offset = {"x": 0, "y": 0, "angle": 0}
        self.last_odometry = {"x": 0, "y": 0, "angle": 0, "time": time.time(), "frame_id": "odom_frame"}
        self.body = body.bodycontroller.BodyController()
        self.config = self.body.get_config()
        self.waypoint_skip_time = None
        self.detections = []
        self.VERSION = 2.0

        # ROS node initialization
        self.ros_node_name = "Brain"
        self.ros_alice_odometry_topic = "odom"
        self.ros_alice_status_topic = "alice_status"
        self.ros_alice_twist_command_topic = "cmd_vel"
        #self.ros_alice_command_topic = "alicecontroller"
        #self.ros_output_topic = "odom"
        self.use_ros = ROS_ENABLED

        if not self.ros_init_topic():
            self.logger.error("ROS import succeeded, but setting up "    \
                                "publisher failed. Is roscore running?")
            self.use_ros = False

        self.transformer = tf.TransformListener()

        if self.use_ros and not self.ros_init_subscription():
            self.logger.error("ROS publishing enabled, but ROS subscription failed.")

        #Simple subscriber to indicate emergency state of the volksbot (TODO:should be done using ROS<->Memory interface?)).
        rospy.Subscriber("/emergency", Bool, self.emergency_callback)

    
    def stop(self):
        print "Stopping Alice connector"

        self.ros_init_publisher.unregister()
        self.status_subscription.unregister()
        self.odo_subscription.unregister()
        self.ros_subscription.unregister()
        self.use_ros = False
        self.logger.info("Unregistering ROS node %s" % self.ros_node_name)

    def ros_init_topic(self):
        try:
            rospy.init_node(self.ros_node_name)
            
            return True
        except:
            traceback.print_exc()
            return False

    def ros_init_subscription(self):
        try:
            self.odo_subscription = rospy.Subscriber(self.ros_alice_odometry_topic, 
                                                     Odometry,
                                                     self.ros_odometry_cb)
            self.status_subscription = rospy.Subscriber(self.ros_alice_status_topic, 
                                                        String,
                                                        self.ros_status_cb)

            return True
        except:
            traceback.print_exc()
            self.ros_subscription = None
            return False

    def ros_odometry_cb(self, odometry):
        if rospy.is_shutdown():
            self.logger.warning("ROSPY is shutting down. Disabling ROS publishing")
            self.use_ros = False
        
        #Converting from Meters to Millimeter, compatibitliy with old pioneer style 
        x = odometry.pose.pose.position.x * 1000
        y = odometry.pose.pose.position.y * 1000
        orientation = tf.transformations.euler_from_quaternion(odometry.pose.pose.orientation)
        theta = orientation[2] / (math.pi / 180.0)
                    
        pose = {'x': x, 'y': y, 'angle': theta}
        pose['battery_level'] = 24 #Alice does not have battery level yet 
        pose['time'] = time.time()
        
        command = {'name':'odometry', 'time': time.time(), 'property_dict': pose}
         
        self.detections.append(command)
        
        try:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "base_link"
            pose.pose = Pose(Point(0, 0, 0), Quaternion())
    
            translation, rotation = self.transformer.lookupTransform("map", "base_link", rospy.Time(0))
    
            orientation = tf.transformations.euler_from_quaternion(rotation)
            new_theta = orientation[2] / (math.pi / 180.0)
            new_odo = {"x": translation[0] * 1000.0,
                       "y": translation[1] * 1000.0, 
                       "angle": new_theta}
            command = {'name':'odometry_corrected', 'time': time.time(), 'property_dict': new_odo}
         
            self.detections.append(command)
        except:
            pass

    def ros_status_cb(self, data):
        #TODO: Deal with Emergency press
        pass
        

    def head_direction(self, target, useAvoidance=False, verbose=False, turnSpeed=1, door = False, move = True):
        """
        Set the target where the alice should go to.
        format = (speed, angle)
            speed in range 0 to 1
            positive values are to the right
            angle in range -180 to 180, where +-180 is backwards and 0 is forward
        This is the function behaviors should call when they want the robot
        to go to some direction.
        """

        if useAvoidance:
            endVector = self.obstacleAvoider.avoid(target) # endVector (length, direction)
        else:
            endVector = target

        if move:
            pass #add cmd_vel
            
        return endVector

    def forward(self, distance):
        """
        Forward in mm. Take care, this will continue until receives a new command or is finished
        DOES NOT USE OBSTACLE AVOIDANCE!
        """
        if self.use_ros:
            self.alice_publisher.publish("forward "+str(distance))

    def turn(self, angle):
        """
        Turn in degrees. Take care, this will continue until receives a new command or is finished
        left is positive
        DOES NOT USE OBSTACLE AVOIDANCE!
        """
        if self.use_ros:
            self.alice_publisher.publish("turn "+str(angle))
    

    def stop_robot(self):
        ''' Send command to stop the robot'''
        #TODO: Add cmd_vel publish
        if self.use_ros:
            self.alice_publisher.publish("stop")

    def is_emergency(self):
        return self.emergency

    def emergency_callback(self, data):
        self.emergency = data
        
    def update(self):
        if self.emergency:
            self.detections.append("EMERGENCY")

        if self.obstacleAvoider and isinstance(self.obstacleAvoider, ObstacleAvoider):
            self.obstacleAvoider.do_visualize()
        detections = self.detections
        self.detections = []
        return detections
