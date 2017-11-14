import time
import string
import traceback
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import logging
import util.nullhandler
import numpy
import math

ROS_ENABLED=False
try:
    import roslib; #roslib.load_manifest('borg_pioneer')
    import rospy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
    import tf
    ROS_ENABLED = True
except:
    print "ROS IS NOT ENABLED"
    pass

logging.getLogger('Borg.Brain.BodyController.MoveBase').addHandler(util.nullhandler.NullHandler())

def distance(t1, t2):
    """ Given two PoseStamped's, determine the distance """
    out = 0
    for dimension in ('x', 'y', 'z'):
        out += math.pow(getattr(t1.pose.position, dimension) - getattr(t2.pose.position, dimension), 2)
    return math.sqrt(out)

class MoveBase(object):
    """ Move_base client. Connects to the move_base action server and gives navigation commands """

    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.BodyController.MoveBase')
        self.logger.setLevel("INFO")
        self.detections = []
        self.use_ros = ROS_ENABLED
        
        self.current_goal = None        # Goal representation as PoseStamped
        self.error_range = -1           # Error range, -1 triggers default error range of movebase
        self.goal_in_range = False      # Is the goal in range
        
        if self.use_ros:
            # Creates the SimpleActionClient
            self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            # Waits until the action server has started up 
            self.logger.info("Waiting for move_base action server")
            self.nav_client.wait_for_server()
            self.logger.info("move_base action server found!")
        else:
            self.logger.warning("Not publishing to ROS as ROS modules cannot "\
                                "be imported")
    
    def update(self):
        detections = self.detections
        self.detections = []
        return detections
    
    # Set error range, how far the robot can be from its destination and still considers to be there
    def set_error_range(self, error_range):
        self.error_range = error_range

    # Set new goal
    def set_new_goal(self, location):
        ''' Sends a goal location to move_base action server
            Goal location should be a dict containing x, y, angle '''
        
        # Create a goal to send to the action server.
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = location["x"]
        goal.target_pose.pose.position.y = location["y"]
        theta = location["angle"] * (math.pi / 180.0)
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        self.current_goal = goal.target_pose
        
        # Sends the goal to the action server.
        self.nav_client.send_goal(goal, done_cb=self.nav_done, feedback_cb=self.nav_feedback)
        print "Navigating towards " + str(location)
        
        # Add detection
        dict = {}
        dict["x"] = location["x"]
        dict["y"] = location["y"]
        dict["angle"] = location["angle"]
        command = {'name':'move_base', 'time':time.time(), 'property_dict':{'new_goal_received': dict}}  
        self.detections.append(command)
        
    # Callback for navigation results
    def nav_done(self, status, result):
        # Check end state of goal
        dict = {}

        if status is GoalStatus.RECALLED:
            dict['status'] = "goal_cancelled"
        elif status is GoalStatus.SUCCEEDED:
            dict['status'] = "goal_reached"
        elif status is GoalStatus.REJECTED:
            dict['status'] = "goal_rejected"
        elif status is GoalStatus.ABORTED:
            dict['status'] = "goal_cancelled"
        else:
            dict['status'] = "goal_cancelled"
        
        
        # When goal is in range, the goal is cancelled, but goal_reached should be returned
        if self.goal_in_range:
            dict['status'] = "goal_reached"
                       
        # Add detection
        command = {'name':'move_base', 'time':time.time(), 'property_dict': dict}  
        self.detections.append(command)
        
        # Reset goal
        self.reset_goal()
        
    # Callback for navigation feedback
    def nav_feedback(self, feedback):
        pose_stamp = feedback.base_position
        
        # Cancels goal when current pose is 'close enough' to goal
        if self.current_goal and self.error_range != -1 and distance(self.current_goal, pose_stamp) < self.error_range:
            self.goal_in_range = True
            self.cancel_goal()
            
    # Cancel a goal
    def cancel_goal(self):
        self.nav_client.cancel_goal()
        
    # Reset goal state
    def reset_goal(self):
        self.current_goal = None
        self.goal_in_range = False




