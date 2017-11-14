'''
Created on Feb 26, 2014

@author: borg
'''

import rospy



#Returns rospy time, perfect when simulator is used

global init


def rostime():
    return rospy.Time().now().to_time()

def init():
    if init:
        return
    else:
        rospy.init_node("utilTime")
