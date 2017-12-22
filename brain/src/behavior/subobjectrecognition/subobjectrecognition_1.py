from __future__ import print_function

import os

import actionlib
import basebehavior.behaviorimplementation
import rospy
from recognition_server.msg import *


class SubObjectRecognition_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        if not hasattr(self, 'command'):
            self.command = 0

        self.client = actionlib.SimpleActionClient("image_server", ProcessAction)
        self.state = 'running'
        pass

    def implementation_update(self):
        if self.state == 'running':
            try:
                rtn = ProcessGoal()
                rtn.state = self.command
                self.client.send_goal(rtn)

                self.state = 'waiting'
                self.now = rospy.get_time()
            except Exception as e:
                self.enablePrint()
                print(e)
                print('Sending goal failed!')
                self.blockPrint()

        elif self.state == 'waiting' and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            self.enablePrint()
            print(result.result.rjust(10), '%', ' ' * 10, result.obj.rjust(10))
            self.blockPrint()
            self.set_finished()

        elif self.state == 'waiting' and self.client.get_state() == actionlib.GoalStatus.ABORTED:
            self.enablePrint()
            print('Failed! Cannot find object in image.')
            self.blockPrint()
            self.client.cancel_all_goals()
            self.state = 'running'
            self.set_failed("ERROR")

        elif self.state == 'waiting' and rospy.get_time() - self.now > 10:
            self.enablePrint()
            print('Timeout!')
            self.blockPrint()
            self.client.cancel_all_goals()
            self.state = 'running'
            self.set_failed("ERROR")

        pass

    # Disable
    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')

    # Restore
    def enablePrint(self):
        sys.stdout = sys.__stdout__
