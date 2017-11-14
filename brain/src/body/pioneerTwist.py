#TODO: create unit test

import socket
import time
import string
import math
import traceback

from navigation.obstacleavoidance.obstacleavoider import ObstacleAvoider
import logging
import util.nullhandler
import body.bodycontroller

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


ROS_ENABLED=False
try:
    import roslib; roslib.load_manifest('borg_pioneer')
    import rospy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
    import tf
    ROS_ENABLED = True
except:
    print "ROS IS NOT ENABLED"
    pass

logging.getLogger('Borg.Brain.BodyController.Pioneer').addHandler(util.nullhandler.NullHandler())

class PioneerTwist(object):
    """
    Controls a pioneer robot.
    (requires the motionController to run on the pioneer)
    """

    def __init__(self, ip_address, port, start_pose = False):
        self.logger = logging.getLogger('Borg.Brain.BodyController.Pioneer')
        self.__ip_address = ip_address
        self.__port = int(port)
        self.logger.info("Connecting to %s:%d" % (self.__ip_address, self.__port))
        self.obstacleAvoider = ObstacleAvoider()
        self.emergency = False
        self.maxSpeed = 400
        self.doormaxSpeed = 500
        self.odo_offset = {"x": 0, "y": 0, "angle": 0}
        self.last_odometry = {"x": 0, "y": 0, "angle": 0, "time": time.time(), "frame_id": "odom_frame"}
        self.body = body.bodycontroller.BodyController()
        self.config = self.body.get_config()
        self.waypoint_skip_time = None
        self.detections = []
        self.VERSION = 1.0

        # ROS node initialization
        self.ros_node_name = "Brain"
        self.ros_input_topic = "amcl_pose"
        self.ros_pioneer_odometry_topic = "pioneer_odometry"
        self.ros_pioneer_status_topic = "pioneer_status"
        self.ros_pioneer_twist_command_topic = "cmd_vel"
        self.ros_pioneer_command_topic = "pioneercontroller"
        self.ros_init_pose_topic = "initialpose"
        self.ros_output_topic = "odom"
        
        self.ros_odom_sim_topic = "odom";
        
        
        self.new_x = 0;
        self.new_y = 0;
        self.new_angle = 0;
        self.last_x = 0;
        self.last_y = 0;
        self.last_angle = 0;
        self.goal_angle = 0;

        self.moveForward = False;
        self.turnBody = False;        
        
        self.use_ros = ROS_ENABLED

        if self.use_ros:
            if not self.ros_init_topic():
                self.logger.error("ROS import succeeded, but setting up "    \
                                    "publisher failed. Is roscore running?")
                self.use_ros = False

            self.transformer = tf.TransformListener()

            if self.use_ros and not self.ros_init_subscription():
                self.logger.error("ROS publishing enabled, but ROS subscription failed.")
        else:
            self.logger.warning("Not publishing to ROS as ROS modules cannot "  \
                               "be imported")

        if start_pose and len(start_pose) == 3:
            self.set_odometry_offset(float(start_pose[0]), float(start_pose[1]), float(start_pose[2]))
            odo = {'x': self.odo_offset['x'],
                   'y': self.odo_offset['y'],
                   'angle': self.odo_offset['angle']}
            self.ros_publish_initial_pose(odo)
            self.last_odometry = {
                "x": self.odo_offset['x'], 
                "y": self.odo_offset['y'], 
                "angle": self.odo_offset['angle'], 
                "time": time.time(), 
                "frame_id": "odom_frame"
            }

    def stop(self):
        print "Stopping Pioneer connector"

        if self.use_ros:
    #        self.ros_publisher.unregister()
            self.pioneer_publisher.unregister()
            self.twist_publisher.unregister()
            self.ros_init_publisher.unregister()
            self.status_subscription.unregister()
            self.odo_subscription.unregister()
            self.ros_subscription.unregister()
            self.use_ros = False
            self.logger.info("Unregistering ROS node %s" % self.ros_node_name)
        else:
            self.logger.info("ROS publishing disabled, so not unregistering")

    def ros_init_topic(self):
        try:
            self.pioneer_publisher = rospy.Publisher(self.ros_pioneer_command_topic, String)
            self.twist_publisher = rospy.Publisher(self.ros_pioneer_twist_command_topic, Twist)
         #   self.ros_publisher = rospy.Publisher(self.ros_output_topic, Odometry)
            self.ros_init_publisher = rospy.Publisher(self.ros_init_pose_topic, PoseWithCovarianceStamped)
            rospy.init_node(self.ros_node_name)
            self.logger.info("Starting publishing to ROS on ROS-topic %s as " \
                             "node %s" % (self.ros_output_topic, self.ros_node_name))
            return True
        except:
            traceback.print_exc()
            return False

    def ros_init_subscription(self):
        try:
            self.odo_subscription = rospy.Subscriber(self.ros_pioneer_odometry_topic, 
                                                     String,
                                                     self.ros_odometry_cb)
            self.status_subscription = rospy.Subscriber(self.ros_pioneer_status_topic, 
                                                        String,
                                                        self.ros_status_cb)
            self.ros_subscription = rospy.Subscriber(self.ros_input_topic, 
                                                     PoseWithCovarianceStamped,
                                                     self.ros_subscription_cb)
            
            self.ros_odom_sim_sub = rospy.Subscriber(self.ros_odom_sim_topic, Odometry, self.receiveSimOdom)
            return True
        except:
            traceback.print_exc()
            self.ros_subscription = None
            return False
    
        

    def ros_publish(self, odometry):
        if self.use_ros:
            if rospy.is_shutdown():
                self.logger.warning("ROSPY is shutting down. Disabling ROS publishing")
                self.ros_publisher.unregister()
                self.pioneer_publisher.unregister()
                self.twist_publisher.unregister()
                self.ros_init_publisher.unregister()
                self.use_ros = False
                return odometry

            try:
                # The new pioneer controller publishes the odometry directly
                if self.VERSION < 2.0:
                    #print "Publishing odometry"
                    odo = Odometry()
                    theta = odometry['angle'] * (math.pi / 180.0)
                    quat = tf.transformations.quaternion_from_euler(0, 0,theta)
                    odo.pose.pose = Pose(Point(odometry['x']/1000., odometry['y']/1000., 0.), Quaternion(*quat))
                    time = odometry['time']
                    secs = int(time)
                    nsecs = int((time - secs) * 1000000000)
                    odo.header.stamp = rospy.Time(secs, nsecs)
                    odo.header.frame_id = "odometry_frame"
                    
                    self.ros_publisher.publish(odo)
            except rospy.ROSInterruptException:
                self.logger.warning("ROSPY generated exception. Disabling ROS publishing")
                self.ros_publisher.unregister()
                self.pioneer_publisher.unregister()
                self.twist_publisher.unregister()
                self.ros_init_publisher.unregister()
                self.use_ros = False
                return odometry

            try:
                # Transform the odometry using the AMCL tf message
                pose = PoseStamped()
                pose.header.stamp = rospy.Time(secs, nsecs)
                pose.header.frame_id = "base_link"
                pose.pose = Pose(Point(0, 0, 0), Quaternion())

                translation, rotation = self.transformer.lookupTransform("map", "base_link", rospy.Time(0))

                orientation = tf.transformations.euler_from_quaternion(rotation)
                new_theta = orientation[2] / (math.pi / 180.0)
                new_odo = {"x": translation[0] * 1000.0,
                           "y": translation[1] * 1000.0, 
                           "angle": new_theta}

                # Return the transformed odometry
                return new_odo
            except Exception as e:
                #self.logger.error("Exception (%s) in transform. Probably AMCL is not working (yet)" % repr(e))
                return odometry


    def ros_publish_initial_pose(self, odometry):
        #print odometry
        init_pose = PoseWithCovarianceStamped()
        theta = odometry['angle'] * (math.pi / 180.0)
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        init_pose.pose.pose = Pose(Point(odometry['x']/1000., odometry['y']/1000., 0.), Quaternion(*quat))
        init_pose.pose.covariance = [0.01] * 36
        init_pose.header.stamp = rospy.Time.now()

        if self.use_ros:
            self.ros_init_publisher.publish(init_pose)

    def ros_subscription_cb(self, data):
        """
        This method is called by ROS when new data has been received on the AMCL
        corrected pose topic
        """
        new_x = data.pose.pose.position.x * 1000.0
        new_y = data.pose.pose.position.y * 1000.0
       
        quat = data.pose.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        new_theta = orientation[2]
        new_time = data.header.stamp.secs + (data.header.stamp.nsecs / 1000000000)

        cur_x = self.last_odometry['x']
        cur_y = self.last_odometry['y']
        cur_theta = self.last_odometry['angle']

        #self.odo_offset['x'] = new_x
        #self.odo_offset['y'] = new_y
        #self.odo_offset['angle'] = new_theta
        pose = {'x': new_x, 'y': new_y, 'angle': new_theta}
        pose['battery_level'] = self.last_odometry['battery_level']
        pose['time'] = new_time
        #print repr(pose)
        command = {'name':'odometry', 'time': new_time, 'property_dict': pose}          
        self.detections.append(command)

    def ros_odometry_cb(self, data):
        self.processMessage(data.data)

    def ros_status_cb(self, data):
        lines = data.data.split("\n")
        if not self.emergency:
            for line in lines:
                if line[:9] == "Version: ":
                    val = float(line[9:])
                    if val != self.VERSION:
                        self.VERSION = val
                        self.logger.info("Pioneer controller version %.2f detected" % val)
                if line[:11] == "Emergency: ":
                    val = line[11:]
                    if val == "yes":
                        self.emergency = True
                        break

            # If we changed to an emergency situation, store the last odometry as the new odo offset
            if self.emergency:
                self.logger.info("Storing last odometry %s as odometry offset" % repr(self.last_odometry))
                self.odo_offset['x'] = self.last_odometry['x']
                self.odo_offset['y'] = self.last_odometry['y']
                self.odo_offset['angle'] = self.last_odometry['angle']

    def set_odometry_offset(self, x, y, angle):
        self.odo_offset['x'] = x
        self.odo_offset['y'] = y
        self.odo_offset['angle'] = angle
        self.logger.info("Set odometry offset to %s" % repr(self.odo_offset))

    def get_odometry_offset(self):
        return self.odo_offset

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



    
###        
    def receiveSimOdom(self, odometry):
        self.new_x = odometry.pose.pose.position.x
        self.new_y = odometry.pose.pose.position.y
     #   self.new_angle = odometry.pose.pose.orientation.z;

        quat = odometry.pose.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        self.new_angle = orientation[2]
        
    #    print self.new_x, self.new_y, self.new_angle
    #    print self.new_angle
        pass
    
    
    def forward(self, distance, speed = 0.2):
        
        twist = Twist()
        twist.linear.x = speed;                   # our forward speed
        twist.linear.y = 0;
        twist.linear.z = 0;
               
        twist.angular.x = 0; 
        twist.angular.y = 0;          
        twist.angular.z = 0;  
        
        self.forwardSpeed = speed;
        
        self.last_x = self.new_x;
        self.last_y = self.new_y;
        
        self.driveDistance = distance;

        self.moveForward = True;
        time.sleep(1)
        
        if self.use_ros:
           self.twist_publisher.publish(twist);
           
    def turn(self, angle, speed = 0.2):           
        
        if (angle > 0):
            sign = -1;
            speed = -speed;
        else:
            sign = 1;
            
        angle = -angle;               

        self.last_angle = self.new_angle;
        self.turnDistance = angle * math.pi / 180.0;
        self.turnSpeed = speed;

     #   print 'turnDistance: ', self.turnDistance
     #   print 'last + turn: ', self.last_angle + self.turnDistance

        time.sleep(1);
        if (sign * (self.last_angle + self.turnDistance) <= math.pi):
            self.goal_angle = self.last_angle + self.turnDistance;

        else:
            rest = sign * math.pi - self.last_angle
            self.turnDistance -= rest;
            self.goal_angle = -sign * math.pi + self.turnDistance;


        self.turnBody = True;

        twist = Twist()
        twist.linear.x = 0;  # our forward speed
        twist.linear.y = 0;
        twist.linear.z = 0;
           
        twist.angular.x = 0; 
        twist.angular.y = 0;          
        twist.angular.z = speed;  

        if self.use_ros:
            self.twist_publisher.publish(twist);


    def move(self, keepRunning = True):
       
        twist = Twist()

      

        if (keepRunning == True):
            current_x = self.new_x;
            current_y = self.new_y;
            current_angle = self.new_angle;
           
            if (self.moveForward == True):
                distance = (abs(self.last_x - current_x) + abs(self.last_y - current_y));
           
                if (distance < self.driveDistance):
                    twist.linear.x = self.forwardSpeed;         
                    twist.linear.y = 0;
                    twist.linear.z = 0;
                    
                    twist.angular.x = 0; 
                    twist.angular.y = 0;          
                    twist.angular.z = 0;

                else:
                    self.moveForward = False;
                    
            if (self.turnBody == True):

                distance = (self.last_angle - current_angle)
         #       print current_angle, self.goal_angle
                
                if (current_angle <= self.goal_angle - 0.05 or current_angle >= self.goal_angle + 0.05):
                    twist.linear.x = 0;         
                    twist.linear.y = 0;
                    twist.linear.z = 0;
                    
                    twist.angular.x = 0; 
                    twist.angular.y = 0;          
                    twist.angular.z = self.turnSpeed;
                else:
                    self.turnBody = False;               
           
        else:
            twist.linear.x = 0;            
            twist.linear.y = 0;
            twist.linear.z = 0;
               
            twist.angular.x = 0; 
            twist.angular.y = 0;          
            twist.angular.z = 0; 
            

        if self.use_ros:
            self.twist_publisher.publish(twist);

        if (self.moveForward == True or self.turnBody == True):
            return True;

        if (self.moveForward == False and self.turnBody == False):
            keepRunning = False;
            return False;



    def stop_robot(self):
        ''' Send command to stop the robot'''
        if self.use_ros:
            self.pioneer_publisher.publish("stop")
        
    # The Functions to read odometry from the pioneer 
    def update(self):
        if self.emergency:
            self.detections.append("EMERGENCY")

	if self.obstacleAvoider and isinstance(self.obstacleAvoider, ObstacleAvoider):
            self.obstacleAvoider.do_visualize()
        detections = self.detections
        self.detections = []
        return detections

    def processMessage(self, msg):
        """
        Process a message received from the pioneer controller.
        This can either be a notification that the emergency
        button has been pressed (or the pioneer is not available)
        or the odometry data.
        """
        
        if not msg:
            return
       
        #self.logger.debug("This is the received string: %s" % msg) 
        #print "PROCESS MESSAGE"
        value_list = msg.split()
        if "EMERGENCY" in value_list:
            if not self.emergency:
                self.logger.info("Storing last odometry %s as odometry offset" % repr(self.last_odometry))
                self.odo_offset['x'] = self.last_odometry['x']
                self.odo_offset['y'] = self.last_odometry['y']
                self.odo_offset['angle'] = self.last_odometry['angle']

            
            self.emergency = True
            return

        # Assume odometry data
        # Remove multiple occurrences of the space characters:
        correct_value_list = filter(None, value_list)
        
        dict = {}
        
        
        #Checking Pioneer returned value numbers
        if len(correct_value_list) == 4:
	    self.logger.debug("The time is not from Pioneer")
            dict["time"] = time.time()
        elif len(correct_value_list) == 5:
            dict["time"] = float(value_list[4])
        else:
            self.logger.warn("Pionner returning less than 4 values")
            return
        x = float(value_list[0])
        y = float(value_list[1])
        angle_rad = math.radians(self.odo_offset['angle'])
        
        
        # Apply rotation by offset angle

        # The new pioneer controller publishes the odometry directly
        if self.VERSION < 2.0:
            dict["x"] = x * math.cos(angle_rad) - y * math.sin(angle_rad)
            dict["y"] = x * math.sin(angle_rad) + y * math.cos(angle_rad)

            # Translate point
            dict["x"] += self.odo_offset["x"]
            dict["y"] += self.odo_offset["y"]
        
            # Update angle
            dict["angle"] = float(value_list[2]) + self.odo_offset['angle']
        else:
            dict["x"] = x
            dict["y"] = y
            dict["angle"] = float(value_list[2])

        # Add battery level
        dict["battery_level"] = float(value_list[3])
        #dict["time"] = float(value_list[4])
        #dict["time"] = time.time()
        
        dict["frame_id"] = self.last_odometry['frame_id']
        
        

        self.last_odometry = dict
        
        command = {'name':'odometry', 'time':time.time(), 'property_dict':dict}
            

        self.emergency = False
        if self.use_ros:
            transformed = self.ros_publish(self.last_odometry)
            command['property_dict'] = transformed

        self.detections.append(command)
