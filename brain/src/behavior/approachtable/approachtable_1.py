import actionlib
import basebehavior.behaviorimplementation
import rospy
from alice_msgs.msg import *


class ApproachTable_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.client = actionlib.SimpleActionClient("aliceapproach", aliceapproachAction)
        self.state = "connect"

    def implementation_update(self):

        if (self.state == "connect"):
            if (self.client.wait_for_server(rospy.Duration(0.1))):
                self.state = "send"
            else:
                print 'Could not connect to alice approach server!'

        elif (self.state == "send"):
            goal = aliceapproachGoal()
            goal.plane = False

            self.client.send_goal(goal)
            self.state = "wait"

        elif (self.state == "wait" and self.client.get_state() == actionlib.GoalStatus.ABORTED):  # something went wrong
            result = self.client.get_result()
            self.set_failed("something went wrong")

        elif (self.state == "wait" and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED):
            result = self.client.get_result()
            self.set_finished()
