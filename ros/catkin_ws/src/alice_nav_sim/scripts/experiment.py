#!/usr/bin/env python

import sys

import time
import rospy
import gazebo as gz
import random

from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

import rospkg

from os.path import *

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
path = rospack.get_path('alice_description')

#TODO: Make this configurable:
#door_sdf_location = join(path,"sdf/door.sdf")
door_sdf_location = join(path,"sdf/fixed_door_nocontact.sdf")
fixed_door_sdf_location = join(path,"sdf/fixed_door_nocontact.sdf")

print door_sdf_location
print fixed_door_sdf_location

sdf_location = {
    "box":'0', 
    "person":'0', 
}

#door_positions = {
    #"door_1":{
        #"closed":       (2.25, 5.00, 0.0000),
        #"half_open":    (2.35, 5.35,-0.5000),
        #"open":         (2.95, 5.75,-1.5708),
    #},
    #"door_2":{
        #"closed":       (5.25, 5.00, 0.0000),
        #"half_open":    (5.35, 5.35,-0.5000),
        #"open":         (5.95, 5.75,-1.5708),
    #},
    #"door_3":{
        #"closed":       (8.25, 5.00, 0.0000),
        #"half_open":    (8.35, 5.35,-0.5000),
        #"open":         (8.95, 5.75,-1.5708),
    #},
#}

door_offset = 0.025
#X, Y:
generic_door_positions = [
    [3 - door_offset, 5.0],
    [6 - door_offset, 5.0],
    [9 - door_offset, 5.0],
]
door_positions = {
    "door_1":{
        "closed":       (generic_door_positions[0][0], generic_door_positions[0][1], 0.0000),
        "half_open":    (generic_door_positions[0][0], generic_door_positions[0][1],-0.5000),
        "open":         (generic_door_positions[0][0], generic_door_positions[0][1],-1.5708),
    },
    "door_2":{
        "closed":       (generic_door_positions[1][0], generic_door_positions[1][1], 0.0000),
        "half_open":    (generic_door_positions[1][0], generic_door_positions[1][1],-0.5000),
        "open":         (generic_door_positions[1][0], generic_door_positions[1][1],-1.5708),
    },
    "door_3":{
        "closed":       (generic_door_positions[2][0], generic_door_positions[2][1], 0.0000),
        "half_open":    (generic_door_positions[2][0], generic_door_positions[2][1],-0.5000),
        "open":         (generic_door_positions[2][0], generic_door_positions[2][1],-1.5708),
    },
}


def generate_state_list_list(possible_state_list = ["open", "half_open", "closed"], size = 3, state_list_list = [], state_list = None):
    """
    Generates all possible combinations of the provided possible_state_list with the specified size.
    """
    if state_list == None:
        state_list = [0] * size
        state_list_list.append(map(lambda x: possible_state_list[x], state_list))
    #Ripple:
    carry_over = True
    for i in range(size):
        state_list[i] += 1 if carry_over else 0
        #Carry over?
        carry_over = state_list[i] == len(possible_state_list)
        if carry_over:
            state_list[i] = 0
    state_list_list.append(map(lambda x: possible_state_list[x], state_list))
    #Recursive call untill finished:
    if len(state_list_list) < pow(len(possible_state_list), size):
        generate_state_list_list(possible_state_list, size, state_list_list, state_list)
    return state_list_list


def delete_doors(amount = 3):
    for i in range(1, amount + 1):
        gz.delete_model("door_%d" % i)

def random_doors(fixed_list = None, noise = 0.0):
    insert_doors(fixed_list, noise)

def insert_doors(fixed_list = None, noise = 0.0):
    """
    Inserts randomdoors and returns the final positions.
    @param  fixed   Uses specified fixed list of door states if specified.
    """

    position_list = []
    for i in range(1, 4):
        #Randomly open, half open or fixed closed door:
        door_state = random.choice(["closed", "half_open", "open", "open"]) if fixed_list == None else fixed_list[i - 1]
        position_list.append(door_state)
        sdf_location = door_sdf_location if not door_state == "closed" else fixed_door_sdf_location 
        x, y, yaw = door_positions["door_%d" % i][door_state]
        if noise > 0.0 and door_state == "half_open":
            yaw += random.uniform(-noise, noise)
        gz.insert_sdf(sdf_location, "door_%d" % i, x, y, yaw)
    return position_list



class TopicSetter(object):
    """
    TODO:XXX: Doesn't seem to work well since topic is not send to all subscribers after the first publish.
    """

    def __init__(self, topic, msg_type, msg):
        self.__topic = topic
        self.__msg_type = msg_type
        self.__msg = msg
        self.__pub = rospy.Publisher(self.__topic, self.__msg_type, queue_size = 1)
        self.__is_set = False
        self.__r = rospy.Rate(1000)
        rospy.Subscriber(self.__topic, self.__msg_type, self.callback)

    def callback(self, data):
        print data
        if data == self.__msg:
            self.__is_set = True

    def update(self):
        self.__pub.publish(self.__msg)
        self.__r.sleep()
        return self.__is_set

def latch_msg(topic, msg_type, msg, duration):
    r = rospy.Rate(1000)
    pub = rospy.Publisher(topic, msg_type, queue_size = 1)
    start_time = rospy.get_time()
    while (rospy.get_time() - start_time) < duration:
        pub.publish(msg)    
        r.sleep()     

def wait_for_ros_time(timeout = 60.0):
    wait_start_time = time.time()
    while True:
        if rospy.get_time() > 0.001:
            break
        if (time.time() - wait_start_time) > timeout:
            self.log.info("No ROS time received for %f seconds, not running any behaviors..." % timeout)
            wait_start_time = time.time()


def stop_robot(duration = 1.0):
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    latch_msg("/cmd_vel", Twist, msg, duration)


def set_topic(topic, msg_type, msg):
    pub = rospy.Publisher(topic, msg_type, queue_size = 1)
    pub.publish(msg)


def stop_move_base_goals():
    pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size = 1)
    msg = GoalID()
    pub.publish(msg)

def add_ground_truth(sample_data, gt_data):
    idx = 0
    pre_gt = None
    for gt in gt_data:
        exp_time = gt["time"]
        while True:
            if idx == len(sample_data):
                break
            sample = sample_data[idx]
            if sample["fe_time"] < exp_time:
                if pre_gt == None:
                    raise "Can't retrieve ground truth state!!"
                sample["_truth"] = pre_gt
                idx += 1
            else:
                break
        pre_gt = gt

