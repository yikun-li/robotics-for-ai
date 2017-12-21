'''
this is an automatically generated template, if you don't rename it, it will be overwritten!
'''
import random

import actionlib
import basebehavior.behaviorimplementation
import rospy
from recognition_server.msg import *


class ObjectRecognition_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.state = 'running'
        self.client = actionlib.SimpleActionClient("image_server", ProcessAction)
        pass

    def implementation_update(self):

        if self.state == 'running':
            self.min = random.randint(0, 255)
            self.max = random.randint(self.min, 255)

            try:
                rtn = ProcessGoal()
                rtn.min = self.min
                rtn.max = self.max
                self.client.send_goal(rtn)

                self.state = 'waiting'
                self.now = rospy.get_time()

            except Exception as e:
                print e
                print 'Sending goal failed!'

        elif self.state == 'waiting' and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            # print result

            if result.sum < 0:
                self.set_failed("ERROR, Sum < 0")
            else:
                print 'Amount of the pixes in duration %d-%d is %d.' % (self.min, self.max, result.sum)
                self.set_finished()

        elif self.state == 'waiting' and rospy.get_time() - self.now > 5:
            self.client.cancel_all_goals()
            self.state = 'running'
            print 'Timeout! Restart the request.'

        pass
