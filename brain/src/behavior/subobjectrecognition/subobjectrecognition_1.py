from __future__ import print_function

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
                print(e)
                print('Sending goal failed!')

        elif self.state == 'waiting' and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            print(result.result.rjust(10), '%', ' '*10, result.obj.rjust(10))
            self.set_finished()

        elif self.state == 'waiting' and rospy.get_time() - self.now > 30:
            print('Timeout!')
            self.client.cancel_all_goals()
            self.state = 'running'
            self.set_failed("ERROR")

        pass
