import roslib
roslib.load_manifest('head_controller')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import time
from geometry_msgs.msg import Twist
from dynamixel_controllers.srv import SetSpeed

class Joint:
    def __init__(self):       
        '''
        self.jta = actionlib.SimpleActionClient('/f_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        '''
        
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callbackCmd_vel, queue_size=1)
        
        # publishers for the pan and tilt servos
      #  self.tiltSpeed = rospy.Publisher("/tilt_controller/set_speed", Float64, queue_size = 1)
      #  self.panSpeed = rospy.Publisher("/pan_controller/set_speed", Float64, queue_size = 1)
        '''
        # Service for setting the speed
        rospy.loginfo('Waiting for set_speed service...')
        rospy.wait_for_service('alice/tilt_controller/set_speed')
        rospy.wait_for_service('alice/pan_controller/set_speed')
        rospy.loginfo('set_speed service found!')
        
        self.setTiltSpeed = rospy.ServiceProxy('alice/tilt_controller/set_speed', SetSpeed)        
        self.setPanSpeed = rospy.ServiceProxy('alice/pan_controller/set_speed', SetSpeed)
        
        # The speed at which the servos should move
        self.tiltSpeed = 0.5
        self.panSpeed = 0.5
        
        # Set the speed values
        self.setTiltSpeed(self.tiltSpeed)
        self.setPanSpeed(self.panSpeed)
   	'''
        self.tiltPosition = rospy.Publisher("alice/tilt_controller/command", Float64, queue_size = 1)
        self.panPosition = rospy.Publisher("alice/pan_controller/command", Float64, queue_size = 1)
        
        self.wasZero = False
        self.lastMove = (0, 0)
        
        # If difference between positions is less than the following numbers, the head won't move.
        self.minTiltDifference = 0
        self.minPanDifference = 0
        
    def callbackCmd_vel(self, data):
        maxPitch = 0.40
        min_pitch = 0.40
        min_pitch_turn = 0.5

        maxPan = 0.50
        maxVelocity = 0.3
        maxTurn = 0.6 
        panBackward = 1.57 # position when driving backwards and rotation
        
        tilt_nav = 0
        pan_nav = 0

        if data.linear.x >= 0: # for forward movement or stopped            
        
            convert = maxPitch / maxVelocity
            tilt_nav = max(convert * data.linear.x, min_pitch)

            convert = maxPan / maxTurn
            pan_nav = convert * data.angular.z

            if not self.difference_enough(tilt_nav, pan_nav):
                return
              
            # Movement ended completely
            if data.linear.x == 0:  
                if self.wasZero == False: 
                    self.wasZero = True
                    self.move_joint([tilt_nav, pan_nav])
            # Moving forward
            else:
                self.wasZero = False
                self.move_joint([tilt_nav, pan_nav])
            
        else: # moving backwards
            self.wasZero = False
            tilt_nav = min_pitch_turn
            
            if data.angular.z > 0:
                pan_nav = -panBackward
                if not self.difference_enough(tilt_nav, pan_nav):
                    return
                self.move_joint([tilt_nav, pan_nav])
            elif data.angular.z < 0:
                pan_nav = panBackward
                if not self.difference_enough(tilt_nav, pan_nav):
                    return
                self.move_joint([tilt_nav, pan_nav])
            # There are no more movement commands sent
            else:
                self.move_joint([tilt_nav, 0])
                 
        #print('{0}, {1}'.format(tilt_nav, pan_nav))# - self.lastMove[1]))
                
    def difference_enough(self, tilt, pan): return True
        # tilt, pan = abs(tilt), abs(pan)
        #return (abs(tilt - self.lastMove[0]) > self.minTiltDifference) and \
        #        (abs(pan - self.lastMove[1]) > self.minPanDifference)
                
        
    def move_joint(self, angles):
        # The speed values
     #   self.tiltSpeed.publish(speed[0])
     #   self.panSpeed.publish(speed[1])
        
        # The positions in radian
       # print "move joint"
        self.lastMove = [abs(x) for x in angles]
        self.tiltPosition.publish(angles[0]) # publish the tilt position
        self.panPosition.publish(angles[1]) # publish the pan position
        '''
        goal = FollowJointTrajectoryGoal()                      
        goal.trajectory.joint_names = ['tilt_joint', 'pan_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(0.01)                       
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)
        '''
        

if __name__ == '__main__':
    rospy.init_node('head_controller')
    head = Joint()
    
    rospy.spin()
    
