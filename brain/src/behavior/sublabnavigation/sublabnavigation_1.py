import math

import basebehavior.behaviorimplementation
import rospy
import tf


class SubLabNavigation_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):
        self.history = []
        self.transform = tf.TransformListener()
        pass

    def implementation_update(self):
        self.record_history()

        if (self.is_stuck()):
            self.set_failed('')
        pass

    def record_history(self):
        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        self.history.append(trans)

    def is_stuck(self):
        self.transform.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = self.transform.lookupTransform('/map', '/base_link', rospy.Time(0))

        if len(self.history) <= 200:
            return False

        cursor = -200
        distance = math.pow((self.history[cursor][0] - trans[0]), 2) + math.pow((self.history[cursor][1] - trans[1]), 2)

        if distance < 0.3:
            return True
        else:
            return False
